"""The cutter's contact angle along a toolpath, measured from the streamed moves.

Independent of the planner: nothing here reads a plan or a planner's report.
The measure replays an operation's own moves over the stock it started on.

**What is measured.** For a flat end mill of radius r with its tip at height
z, the contact angle at a point of the path is the part of the leading half
of the cutter's rim (the 180 degrees ahead of the direction of travel) that
lies in material. A point of the rim lies in material when

- the stock the operation started on has material above z there (the
  simulated stock's cells when the operation began: exact for an uncut box,
  otherwise known to one stock cell), and
- no earlier move of the operation swept it at or below z: the point is not
  within r of that move's centre line where the move's tip is at or below z.
  Lines, arcs and helices are tested in closed form, not on a grid. (A grid
  cannot resolve this: the rim runs tangent to the slot it has just cut, so
  a cell of r/100 leaves the last 7 degrees beside the cutter undecided; a
  closed-form distance leaves 0.03 degrees.)

**Resolution.** The rim's leading half is sampled at `SAMPLES` points
(`SAMPLE_DEG` apart); the angle is the count of samples in material times
`SAMPLE_DEG`. Each end of a contact arc is therefore placed within half a
sample, so a single arc reads within `SAMPLE_DEG` of its true angle, with no
bias. `OVER_DEG` (two samples) is the margin above a limit before a reading
counts as over it. The path is evaluated at the midpoints of steps of at
most r * `STEP` (never fewer than one per move), each weighted by its
length; steps of r/25 give the same shares to a tenth of a percentage point
on the benchmark's adaptive workloads, at 2.5 times the time. Tolerances
`EPS_R` (relative, on the sweep radius) and `EPS_Z` (mm, on heights) only
decide exact ties.

The reported numbers are sums over a deterministic sequence, but they go
through the platform's sin/cos/atan2: a sample on a boundary may fall either
way on another platform, so compare them, do not require them bit-equal.

Only horizontal cutting, link and entry-feed moves of flat end mills are
measured. Helical entries are not: a helix cuts a full slot by design.
"""

from __future__ import annotations

import math

import numpy as np

SAMPLES = 360
SAMPLE_DEG = 180.0 / SAMPLES
OVER_DEG = 2 * SAMPLE_DEG
STEP = 1 / 10
EPS_R = 1e-7
EPS_Z = 1e-6


def declared_limit_deg(operation, radius):
    """The contact-angle limit an operation declares, acos(1 - w/r) for its
    radial width w, or None when it declares none this measure applies to
    (only stock-following adaptive clearing does)."""
    kind = type(operation).__name__
    if kind == "AdaptivePocket" and operation.path_strategy == "engagement_front":
        width = operation.stepover_mm
        if operation.max_engagement_deg < 90:
            width = min(width, radius * (1 - math.cos(math.radians(operation.max_engagement_deg))))
    elif kind in ("Roughing", "RestRoughing"):
        clearing = operation.clearing
        if (
            type(clearing).__name__ != "AdaptiveClearing"
            or getattr(clearing, "path_strategy", None) != "engagement_front"
        ):
            return None
        width = clearing.max_radial_engagement_mm
    else:
        return None
    return math.degrees(math.acos(1 - width / radius))


class _Moves:
    """An operation's feed moves as arrays (rapids cut nothing: the stream's
    stock-contact check owns that)."""

    def __init__(self, motions):
        rows = []
        for m in motions:
            c = m["curve"]
            (x0, y0, z0), (x1, y1, z1) = c["start"], c["end"]
            if c["kind"] == "line":
                rows.append((0, x0, y0, x1, y1, z0, z1, 0, 0, 0, 0, 0))
            else:
                if c["plane"] != "xy":
                    raise ValueError(f"contact angle is measured in the XY plane; {m['id']} is an arc in {c['plane']}")
                cx, cy = c["center"][:2]
                rows.append(
                    (1, x0, y0, x1, y1, z0, z1, cx, cy, math.hypot(x0 - cx, y0 - cy), math.atan2(y0 - cy, x0 - cx))
                    + (c["sweep_rad"],)
                )
        a = np.array(rows, dtype=float).reshape(len(rows), 12)
        self.arc = a[:, 0] == 1
        self.a, self.b = a[:, 1:3], a[:, 3:5]
        self.z0, self.z1 = a[:, 5], a[:, 6]
        self.centre, self.radius, self.start, self.sweep = a[:, 7:9], a[:, 9], a[:, 10], a[:, 11]
        # A box around each move: an arc's from nine points along it, grown
        # by how far the arc bulges between them.
        along = self.start[:, None] + np.linspace(0, 1, 9)[None] * self.sweep[:, None]
        bulge = self.radius * (1 - np.cos(np.minimum(np.abs(self.sweep) / 16, math.pi)))
        xs = np.where(
            self.arc[:, None],
            self.centre[:, 0, None] + self.radius[:, None] * np.cos(along),
            a[:, [1, 3]].mean(1)[:, None],
        )
        ys = np.where(
            self.arc[:, None],
            self.centre[:, 1, None] + self.radius[:, None] * np.sin(along),
            a[:, [2, 4]].mean(1)[:, None],
        )
        ends_low, ends_high = np.minimum(self.a, self.b), np.maximum(self.a, self.b)
        self.low = np.minimum(ends_low, np.stack([xs.min(1), ys.min(1)], axis=1) - bulge[:, None])
        self.high = np.maximum(ends_high, np.stack([xs.max(1), ys.max(1)], axis=1) + bulge[:, None])

    def swept_parts(self, index, z, low, high):
        """Moves before `index` that come into the box low-high, restricted
        to where their tip is at or below z: lines (a, b) and arcs (see
        `_arcs`)."""
        s = np.flatnonzero((self.low[:index] <= high).all(axis=1) & (self.high[:index] >= low).all(axis=1))
        z0, z1 = self.z0[s], self.z1[s]
        zc = z + EPS_Z
        span = np.where(z0 == z1, 1.0, z1 - z0)
        # The tip height is linear along a line or helix.
        cross = np.clip((zc - z0) / span, 0, 1)
        lo = np.where((z0 > zc) & (z1 < z0), cross, 0.0)
        hi = np.where((z1 > zc) & (z1 > z0), cross, 1.0)
        keep = (np.minimum(z0, z1) <= zc) & (hi >= lo)
        arc = self.arc[s]
        lines = keep & ~arc
        d = self.b[s] - self.a[s]
        a = (self.a[s] + lo[:, None] * d)[lines]
        b = (self.a[s] + hi[:, None] * d)[lines]
        arcs = keep & arc
        return (
            (a, b),
            _arcs(
                self.centre[s][arcs],
                self.radius[s][arcs],
                (self.start[s] + lo * self.sweep[s])[arcs],
                ((hi - lo) * self.sweep[s])[arcs],
            ),
        )


def _arcs(centre, radius, start, sweep):
    """Arcs ready for `_on_arcs`: centre x and y, radius, the unit directions
    of the start and the end from the centre, and the signed sweep."""
    end = start + sweep
    return np.stack(
        np.broadcast_arrays(
            centre[..., 0], centre[..., 1], radius, np.cos(start), np.sin(start), np.cos(end), np.sin(end), sweep
        )
    )


def _line_distance2(points, a, b):
    """Squared distance from each point (M) to each segment a-b (J): (M, J)."""
    dx, dy = (b - a).T
    l2 = dx * dx + dy * dy
    vx = points[:, 0, None] - a[None, :, 0]
    vy = points[:, 1, None] - a[None, :, 1]
    t = vx * dx
    t += vy * dy
    t /= np.where(l2 == 0, 1.0, l2)
    np.clip(t, 0, 1, out=t)
    vx -= t * dx
    vy -= t * dy
    vx *= vx
    vy *= vy
    vx += vy
    return vx


def _on_arcs(px, py, arcs, reach):
    """Whether each point is within `reach` of its arc (the arrays pair up,
    one point with one column of `arcs`): within `reach` of the circle where
    the point's direction from the centre is inside the arc, else within
    `reach` of either end. No angle is computed: with v the point's
    direction, s and e the arc's start and end directions and the arc
    turning counter-clockwise, v is inside an arc of at most half a turn
    when s x v >= 0 and v x e >= 0, and inside a longer one unless both are
    negative (the gap is then under half a turn)."""
    cx, cy, radius, sx, sy, ex, ey, sweep = arcs
    vx, vy = px - cx, py - cy
    sense = np.sign(sweep)
    after = (sx * vy - sy * vx) * sense >= 0
    before = (vx * ey - vy * ex) * sense >= 0
    wide = np.abs(sweep)
    inside = np.where(wide <= math.pi, after & before, (after | before) | (wide >= math.tau))
    near = inside & (np.abs(np.hypot(vx, vy) - radius) < reach)
    for ux, uy in ((sx, sy), (ex, ey)):
        qx, qy = vx - radius * ux, vy - radius * uy
        near |= qx * qx + qy * qy < reach * reach
    return near


def _near_arcs(points, arcs, reach):
    """Whether each point (M) is within `reach` of each arc (J): (M, J)."""
    vx = points[:, 0, None] - arcs[0][None]
    vy = points[:, 1, None] - arcs[1][None]
    vx *= vx
    vy *= vy
    vx += vy
    np.sqrt(vx, out=vx)
    vx -= arcs[2]
    np.abs(vx, out=vx)
    # No point of an arc is nearer than the circle it lies on, so only
    # points this close to the circle need the arc's extent looked at.
    near = vx < reach
    i, j = np.nonzero(near)
    if len(i):
        near[i, j] = _on_arcs(points[i, 0], points[i, 1], arcs[:, j], reach)
    return near


# Steps whose earlier moves are narrowed down together (see `contact_angles`).
_GROUP = 16
_RECENT = 4


def contact_angles(motions, radius, stock, heights, measured=("cut", "link", "lead")):
    """For each horizontal move of a kind in `measured`: (index into
    `motions`, step lengths, contact angle in degrees at each step).

    `motions`: the operation's motion dicts in stream order. `stock`: (min
    x, min y, cell dx, cell dy, nx, ny). `heights`: the stock's cells when
    the operation began, as the stream holds them (lower and upper bound per
    cell, row by row)."""
    feeds = [m for m in motions if m["kind"] not in ("rapid", "shift")]
    order = [i for i, m in enumerate(motions) if m["kind"] not in ("rapid", "shift")]
    moves = _Moves(feeds)
    x_min, y_min, dx, dy, nx, ny = stock
    upper = np.asarray(heights, dtype=float)[1::2].reshape(ny, nx)
    theta = np.radians(-90 + (np.arange(SAMPLES) + 0.5) * SAMPLE_DEG)
    rim = radius * np.cos(theta), radius * np.sin(theta)
    reach = radius * (1 - EPS_R)
    out = []
    for index, m in enumerate(feeds):
        if m["kind"] not in measured or moves.z0[index] != moves.z1[index]:
            continue
        z = moves.z0[index]
        a, b = moves.a[index], moves.b[index]
        if moves.arc[index]:
            length = abs(moves.radius[index] * moves.sweep[index])
        else:
            length = math.hypot(*(b - a))
        steps = max(1, math.ceil(length / (radius * STEP)))
        t = (np.arange(steps) + 0.5) / steps
        if moves.arc[index]:
            angle = moves.start[index] + t * moves.sweep[index]
            centres = moves.centre[index] + moves.radius[index] * np.stack([np.cos(angle), np.sin(angle)], axis=1)
            heading = angle + math.copysign(math.pi / 2, moves.sweep[index])
        else:
            centres = a + t[:, None] * (b - a)
            heading = np.full(steps, math.atan2(b[1] - a[1], b[0] - a[0]))
        # Only moves that come within the cutter's diameter of this one's
        # centres can hold a point of its rim.
        (la, lb), earlier = moves.swept_parts(
            index, z, centres.min(axis=0) - 2 * radius, centres.max(axis=0) + 2 * radius
        )
        angles = np.empty(steps)
        for first in range(0, steps, _GROUP):
            k = slice(first, first + _GROUP)
            # Of those, the ones near these steps.
            near = (
                (_line_distance2(centres[k], la, lb) < 4 * radius * radius).any(axis=0)
                if len(la)
                else np.zeros(0, bool)
            )
            a, b = la[near], lb[near]
            arcs = earlier[:, _near_arcs(centres[k], earlier, 2 * radius).any(axis=0)]
            # The rim, turned to each step's heading.
            hx, hy = np.cos(heading[k, None]), np.sin(heading[k, None])
            points = np.empty((len(centres[k]) * SAMPLES, 2))
            points[:, 0] = (centres[k, 0, None] + hx * rim[0] - hy * rim[1]).ravel()
            points[:, 1] = (centres[k, 1, None] + hy * rim[0] + hx * rim[1]).ravel()
            ix = np.floor((points[:, 0] - x_min) / dx).astype(int)
            iy = np.floor((points[:, 1] - y_min) / dy).astype(int)
            inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            material = np.zeros(len(points), dtype=bool)
            material[inside] = upper[iy[inside], ix[inside]] > z + EPS_Z
            # Earlier moves decide only the points still in material.
            live = np.flatnonzero(material)
            # The latest moves first: the pass before this one lies beside
            # it and clears most of what is cleared, so few points go on to
            # the rest.
            for part in (slice(-_RECENT, None), slice(None, -_RECENT)):
                if len(live) and len(a[part]):
                    live = live[~(_line_distance2(points[live], a[part], b[part]) < reach * reach).any(axis=1)]
                if len(live) and arcs[:, part].shape[1]:
                    live = live[~_near_arcs(points[live], arcs[:, part], reach).any(axis=1)]
            if len(live) and moves.arc[index]:
                # This arc's own trail up to each step. (A straight move's
                # trail never reaches its leading half: every point of that
                # half is at least r from the line behind the centre.)
                trail = _arcs(
                    moves.centre[index],
                    moves.radius[index],
                    moves.start[index],
                    np.repeat(t[k] * moves.sweep[index], SAMPLES)[live],
                )
                live = live[~_on_arcs(points[live, 0], points[live, 1], trail, reach)]
            angles[k] = np.bincount(live // SAMPLES, minlength=len(centres[k])) * SAMPLE_DEG
        out.append((order[index], np.full(steps, length / steps), angles))
    return out


def summary(motions, radius, stock, heights, limit_deg=None):
    """The engagement record of one operation (see the module docstring).

    Shares are fractions of the measured cutting length (horizontal moves of
    kind "cut")."""
    result = contact_angles(motions, radius, stock, heights)
    by_kind = {}
    for index, lengths, angles in result:
        by_kind.setdefault(motions[index]["kind"], []).append((lengths, angles))

    def joined(kind):
        parts = by_kind.get(kind, [])
        if not parts:
            return np.zeros(0), np.zeros(0)
        return np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts])

    def share(lengths, mask):
        total = lengths.sum()
        return round(float(lengths[mask].sum() / total), 4) if total else None

    lengths, angles = joined("cut")
    total = float(lengths.sum())
    record = dict(
        cut_mm=round(total, 3),
        mean_deg=round(float((lengths * angles).sum() / total), 2) if total else None,
        max_deg=round(float(angles.max()), 2) if total else None,
        none_share=share(lengths, angles == 0),
    )
    if limit_deg is not None:
        record.update(
            limit_deg=round(limit_deg, 2),
            over_share=share(lengths, angles > limit_deg + OVER_DEG),
            within_10_share=share(lengths, (angles >= limit_deg - 10) & (angles <= limit_deg + OVER_DEG)),
            under_10_20_share=share(lengths, (angles >= limit_deg - 20) & (angles < limit_deg - 10)),
            under_20_share=share(lengths, (angles > 0) & (angles < limit_deg - 20)),
        )
    link_lengths, link_angles = joined("link")
    record.update(
        link_mm=round(float(link_lengths.sum()), 3),
        link_none_share=share(link_lengths, link_angles == 0),
    )
    entry_lengths, entry_angles = joined("lead")
    flat = float(entry_lengths.sum())
    record.update(
        entry_feed_mm=round(flat, 3),
        entry_feed_mean_deg=round(float((entry_lengths * entry_angles).sum() / flat), 2) if flat else None,
    )
    return record
