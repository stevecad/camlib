# Windows

Use 64-bit CPython 3.12 or newer and the `x86_64-pc-windows-msvc`
Rust toolchain (Rust 1.88 or newer). Source builds also require Visual Studio
2022 Build Tools with **Desktop development with C++**, including MSVC and a
Windows SDK. Reopen PowerShell after installing the tools so Cargo is on PATH.
WSL and Docker are not required. Windows ARM64 and 32-bit builds are not covered
by this setup or the Windows CI job.

From the repository root in PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install .
.\.venv\Scripts\python.exe examples/standalone_job.py out
```

Replace `-3.12` with your installed Python version if necessary. Calling the
virtual environment's Python directly avoids PowerShell activation-policy changes.
The package bundles the native extension and `cam-sim-stream.exe`.
CadQuery is optional; install it in the same environment for CAD examples:

```powershell
.\.venv\Scripts\python.exe -m pip install cadquery
.\.venv\Scripts\python.exe examples/cadquery_job.py out-cad
```

## Development

```powershell
$env:PYO3_PYTHON = (Resolve-Path .venv\Scripts\python.exe).Path
cargo build --release --workspace --all-features
$env:PYTHONPATH = 'python'
.\.venv\Scripts\python.exe -m unittest discover -s python/tests
cargo test --release --workspace --all-features --lib --tests
cargo test --release --doc
```

Use a process time and memory limit for long test runs. The persistence tests
exercise real symbolic links; that subtest reports a skip if Windows denies
creation. Enable Developer Mode or use a shell with symlink privilege to run it.
Normal package use does not require this permission.

Build a distributable wheel with:

```powershell
.\.venv\Scripts\python.exe -m pip wheel . --no-deps --wheel-dir dist
$wheel = (Get-ChildItem dist\camlib-*-win_amd64.whl).FullName
.\.venv\Scripts\python.exe -m pip install --force-reinstall $wheel
```

Installing a prebuilt wheel does not require Rust or C++ build tools. CI builds
the Windows wheel, tests its installed package, and uploads it as an artifact.

The benchmark runner works on Windows, but peak RSS measurements are unavailable
because Python's `resource.getrusage` is Unix-only. Its JSON records `null` for
these measurements and reports display `unavailable`; timing and toolpath
measurements are still recorded. macOS RSS values are converted from bytes to KiB.
