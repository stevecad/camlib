//! Bounded greedy scheduling over existing source-admitted strokes and routes.
use super::*;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RasterRouting {
    Economical,
}
#[derive(Debug, Serialize)]
pub struct RoutingReport {
    pub policy: RasterRouting,
    pub work: usize,
    pub candidates: usize,
    pub graph_work: usize,
    pub geometry_work: usize,
    pub reversed_rasters: usize,
    pub boundary_rotations: usize,
    pub component_visits: usize,
    pub scheduled_link_seconds: f64,
}
impl RoutingReport {
    fn new(policy: RasterRouting, work: usize) -> Self {
        Self {
            policy,
            work,
            candidates: 0,
            graph_work: 0,
            geometry_work: 0,
            reversed_rasters: 0,
            boundary_rotations: 0,
            component_visits: 0,
            scheduled_link_seconds: 0.,
        }
    }
}
#[derive(Clone, Copy)]
struct Candidate {
    item: usize,
    rotation: usize,
    stroke: Stroke,
}
#[derive(Default)]
struct Group {
    rasters: Vec<Stroke>,
    loops: Vec<Vec<Stroke>>,
}
struct Scheduler<'a, C: CancelSignal> {
    plan: &'a PocketPlan,
    entries: &'a PocketEntries,
    cancel: &'a C,
    report: RoutingReport,
}
impl<C: CancelSignal> Scheduler<'_, C> {
    fn check(&self) -> Result<(), String> {
        if self.cancel.is_cancelled() {
            Err("raster scheduling cancelled".into())
        } else {
            Ok(())
        }
    }
    fn charge(&mut self, n: usize) -> Result<(), String> {
        self.report.work = self
            .report
            .work
            .checked_add(n)
            .ok_or("raster scheduling work overflow")?;
        Ok(())
    }
    fn pick(
        &mut self,
        from: Point,
        entry: bool,
        candidates: &[Candidate],
    ) -> Result<Candidate, String> {
        self.check()?;
        let before = self.report.work;
        let tree = self
            .entries
            .distances(from, &mut self.report.work, self.cancel)?;
        self.report.graph_work += self.report.work - before;
        let op = &self.plan.operation;
        let entry_feed = op
            .entry
            .as_ref()
            .and_then(|e| e.feed_mm_min())
            .unwrap_or(op.plunge_mm_min);
        let mut best = None::<(f64, Candidate)>;
        for &candidate in candidates {
            self.check()?;
            self.charge(1)?;
            self.report.candidates += 1;
            let length = norm(from, candidate.stroke.a);
            let lower = 60. * length / op.feed_mm_min.max(entry_feed);
            if best.is_some_and(|(cost, _)| lower > cost) {
                continue;
            }
            let mut cost = 60. * tree.distance(self.entries, candidate.stroke.a)? / entry_feed;
            if length == 0. {
                cost = 0.;
            } else if !entry && length <= 4. * op.stepover_mm {
                let work = self.plan.link_work_per_query();
                self.charge(work)?;
                self.report.geometry_work += work;
                if self.plan.link_geometry_fits(from, candidate.stroke.a) {
                    cost = 60. * length / op.feed_mm_min;
                }
            }
            if best.is_none_or(|(old, _)| cost < old) {
                best = Some((cost, candidate));
            }
        }
        let (cost, chosen) = best.ok_or("no economical raster candidate")?;
        self.report.scheduled_link_seconds += cost;
        Ok(chosen)
    }
}
impl PocketPlan {
    /// Apply only to untouched native roughing layers. The option lives in the
    /// authored RoughingClearing, so semantic caches distinguish the policy.
    pub(crate) fn with_routing(
        self,
        policy: RasterRouting,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if self.operation.rest_tolerance_mm.is_some() {
            return Err("raster scheduling needs an untouched non-rest plan".into());
        }
        self.schedule_guides(policy, cancel)
    }
    /// Internal whole-guide scheduling, before rest subdivision. The complete
    /// loops may rotate but never reverse; stock filtering happens at emission.
    pub(super) fn schedule_guides(
        mut self,
        policy: RasterRouting,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("raster scheduling cancelled".into());
        }
        if self.pass != 0
            || self.stroke != 0
            || !self.pending.is_empty()
            || !self.rest_group_ends.is_empty()
        {
            return Err("raster scheduling needs an untouched non-rest plan".into());
        }
        if let Some(entry) = &self.operation.entry {
            entry.refuse_ramp(
                &format!("operation {}", self.operation.id),
                "scheduled (offset or economical raster) clearing links its passes through fitted helical entry routes",
            )?;
        }
        if self.strokes.is_empty() {
            self.report.routing = Some(RoutingReport::new(policy, 0));
            return Ok(self);
        }
        let entries = self
            .entries
            .as_ref()
            .ok_or("economical raster needs admitted entry routes")?;
        let mut groups: Vec<Group> = (0..entries.locations.len())
            .map(|_| Group::default())
            .collect();
        let component = |s: Stroke| -> Result<usize, String> {
            let a = entries.component(s.a)?;
            if entries.component(s.b)? != a {
                return Err("cut stroke crosses entry components".into());
            }
            Ok(a)
        };
        for &s in &self.strokes[..self.report.raster_strokes] {
            if cancel.is_cancelled() {
                return Err("raster scheduling cancelled".into());
            }
            if s.arc.is_some() {
                return Err("raster scheduling expected line strokes".into());
            }
            groups[component(s)?].rasters.push(s);
        }
        let mut i = self.report.raster_strokes;
        while i < self.strokes.len() {
            if cancel.is_cancelled() {
                return Err("boundary scheduling cancelled".into());
            }
            let start = self.strokes[i].a;
            let group = component(self.strokes[i])?;
            let mut ring = vec![];
            loop {
                if cancel.is_cancelled() {
                    return Err("boundary scheduling cancelled".into());
                }
                let s = *self.strokes.get(i).ok_or("boundary loop did not close")?;
                if component(s)? != group || ring.last().is_some_and(|old: &Stroke| old.b != s.a) {
                    return Err("boundary scheduling lost a continuous loop".into());
                }
                ring.push(s);
                i += 1;
                if s.b == start {
                    break;
                }
            }
            groups[group].loops.push(ring);
        }
        let mut scheduler = Scheduler {
            plan: &self,
            entries,
            cancel,
            report: RoutingReport::new(policy, entries.work),
        };
        let mut order = Vec::with_capacity(self.strokes.len());
        let mut cursor = [self.initial[0], self.initial[1]];
        loop {
            scheduler.check()?;
            let group = (0..groups.len())
                .filter(|&i| !groups[i].rasters.is_empty() || !groups[i].loops.is_empty())
                .min_by(|&a, &b| {
                    let point = |i: usize| {
                        let e = &entries.locations[i];
                        [e.center[0] + e.radius, e.center[1]]
                    };
                    norm(cursor, point(a))
                        .total_cmp(&norm(cursor, point(b)))
                        .then(a.cmp(&b))
                });
            let Some(group) = group else { break };
            scheduler.report.component_visits += 1;
            let mut from = entries.root(group);
            let mut entry = true;
            let mut rasters: Vec<_> = std::mem::take(&mut groups[group].rasters)
                .into_iter()
                .map(Some)
                .collect();
            while rasters.iter().any(Option::is_some) {
                let candidates: Vec<_> = rasters
                    .iter()
                    .enumerate()
                    .flat_map(|(item, s)| {
                        s.iter().flat_map(move |&s| {
                            [
                                Candidate {
                                    item,
                                    rotation: 0,
                                    stroke: s,
                                },
                                Candidate {
                                    item,
                                    rotation: 1,
                                    stroke: s.reversed(),
                                },
                            ]
                        })
                    })
                    .collect();
                let chosen = scheduler.pick(from, entry, &candidates)?;
                rasters[chosen.item] = None;
                scheduler.report.reversed_rasters += chosen.rotation;
                from = chosen.stroke.b;
                order.push(chosen.stroke);
                entry = false;
            }
            let mut loops: Vec<_> = std::mem::take(&mut groups[group].loops)
                .into_iter()
                .map(Some)
                .collect();
            while loops.iter().any(Option::is_some) {
                let candidates: Vec<_> = loops
                    .iter()
                    .enumerate()
                    .flat_map(|(item, ring)| {
                        ring.iter().flat_map(move |ring| {
                            ring.iter()
                                .enumerate()
                                .map(move |(rotation, &stroke)| Candidate {
                                    item,
                                    rotation,
                                    stroke,
                                })
                        })
                    })
                    .collect();
                let chosen = scheduler.pick(from, entry, &candidates)?;
                let ring = loops[chosen.item].take().unwrap();
                scheduler.report.boundary_rotations += usize::from(chosen.rotation != 0);
                for j in 0..ring.len() {
                    scheduler.check()?;
                    order.push(ring[(chosen.rotation + j) % ring.len()]);
                }
                from = order.last().unwrap().b;
                entry = false;
            }
            cursor = from;
        }
        scheduler.check()?;
        if order.len() != self.strokes.len() {
            return Err("raster schedule lost cutting strokes".into());
        }
        let report = scheduler.report;
        self.strokes = order;
        self.report.routing = Some(report);
        Ok(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    fn plan() -> PocketPlan {
        let m: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let Operation::Pocket(p) = m.operations[0].clone() else {
            panic!("fixture")
        };
        PocketPlan::prepare(
            p,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .unwrap()
    }
    struct CancelAfter(AtomicUsize);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool {
            // Count down to zero; cancelled once nothing is left.
            let mut left = self.0.load(Ordering::Relaxed);
            loop {
                let Some(next) = left.checked_sub(1) else {
                    return true;
                };
                match self
                    .0
                    .compare_exchange_weak(left, next, Ordering::Relaxed, Ordering::Relaxed)
                {
                    Ok(_) => return false,
                    Err(actual) => left = actual,
                }
            }
        }
    }
    #[test]
    fn scheduler_rejects_budget_and_cancellation_without_mutating_cutting_geometry() {
        let plan = plan();
        let entries = plan.entries.as_ref().unwrap();
        let from = entries.root(0);
        let candidates = [Candidate {
            item: 0,
            rotation: 0,
            stroke: plan.strokes[0],
        }];
        let before: Vec<_> = plan.strokes.iter().map(|s| s.curve(0.)).collect();
        let cancel = AtomicBool::new(false);
        // Work and candidate counts are reported, never capped: large prior
        // counts still allow a pick.
        {
            let mut report = RoutingReport::new(RasterRouting::Economical, 1_000_000_000);
            report.candidates = 1_000_000_000;
            let mut scheduler = Scheduler {
                plan: &plan,
                entries,
                cancel: &cancel,
                report,
            };
            assert!(scheduler.pick(from, false, &candidates).is_ok());
        }
        let cancel = CancelAfter(AtomicUsize::new(2));
        let mut scheduler = Scheduler {
            plan: &plan,
            entries,
            cancel: &cancel,
            report: RoutingReport::new(RasterRouting::Economical, 0),
        };
        assert!(
            scheduler
                .pick(from, false, &candidates)
                .err()
                .expect("cancellation accepted")
                .contains("cancelled")
        );
        assert_eq!(
            plan.strokes.iter().map(|s| s.curve(0.)).collect::<Vec<_>>(),
            before
        );
        assert!(plan.report.routing.is_none());
        assert!(plan.pending.is_empty());
        assert_eq!(plan.next_id, 1);
    }
    #[test]
    fn empty_schedule_reports_the_selected_policy_and_honors_cancellation() {
        let m: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let Operation::Pocket(mut op) = m.operations[0].clone() else {
            panic!("fixture")
        };
        op.region = PlanarRegion::circle("too-small", [16., 12.], 0.5).unwrap();
        let p = PocketPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .unwrap();
        assert!(p.strokes.is_empty());
        let p = p
            .with_routing(RasterRouting::Economical, &AtomicBool::new(false))
            .unwrap();
        let r = p.report.routing.unwrap();
        assert_eq!(r.component_visits, 0);
        assert_eq!(r.work, 0);
        assert!(
            plan()
                .with_routing(RasterRouting::Economical, &AtomicBool::new(true))
                .is_err()
        );
    }
}
