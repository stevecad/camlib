"""Check that a release tag matches every version the repository declares.

    python packaging/check_release.py v0.1.0

The tag must be `v` followed by the version in pyproject.toml, which must also
be the version of both Rust crates and have a section in CHANGELOG.md. Exits
non-zero and says what differs otherwise, so a mistyped tag publishes nothing.
"""

from pathlib import Path
import re
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def problems(tag):
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())["project"]
    version = project["version"]
    found = []
    if tag != f"v{version}":
        found.append(f"tag {tag!r} is not 'v{version}' (pyproject.toml declares {version})")
    for manifest in ("Cargo.toml", "bindings/python/Cargo.toml"):
        crate = tomllib.loads((ROOT / manifest).read_text())["package"]["version"]
        if crate != version:
            found.append(f"{manifest} declares {crate}, pyproject.toml {version}")
    changelog = (ROOT / "CHANGELOG.md").read_text()
    if not re.search(rf"^## {re.escape(version)}\b", changelog, re.MULTILINE):
        found.append(f"CHANGELOG.md has no '## {version}' section")
    return found


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    found = problems(sys.argv[1])
    for line in found:
        print(line, file=sys.stderr)
    sys.exit(1 if found else 0)
