# Changelog

Releases of `cam-engine` (the `camlib` Python package and its Rust engine).
Each GitHub release lists the pull requests merged since the one before; this
file says what a release means for someone using the library.

Versions follow [semantic versioning](https://semver.org). Before 1.0 a minor
version (0.x) may change the Python API, the report fields or the toolpaths an
operation plans; a patch version (0.x.y) does not change the API.

## 0.1.0

The first published release.

- A headless CAM engine for three-axis milling, turning and plasma/laser
  cutting, driven from Python: jobs, setups, tools, machines and
  postprocessors are plain objects.
- Every move is streamed with the stock it removes; runs can be played back,
  cancelled, saved and regenerated from a cache.
- Programs are checked before export: moves against stock, fixtures and
  machine limits, stock against the target part, and each posted program
  decoded again and compared with the plan. A program with findings that
  were not accepted is refused.
- Postprocessors for LinuxCNC, GRBL, FANUC, Haas, Siemens 808D, Centroid,
  LNC, Carvera, QtPlasmaC and WinCNC plasma (see `docs/posts.md`). None has
  been run on a real machine yet: see the safety note in the README.
- Wheels for Linux x86_64 and aarch64 (manylinux_2_28) and Windows x64, for
  CPython 3.12 and later. Other platforms install from the source
  distribution and need a Rust toolchain.
