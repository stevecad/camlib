#!/usr/bin/env bash
# Build the cam-engine wheel (the `camlib` package) for Linux as
# manylinux_2_28 (glibc 2.28 or newer: RHEL 8, Debian 10, Ubuntu 18.10 and
# later) in the PyPA build image, from the committed tree (git archive HEAD:
# nothing uncommitted or ignored).
#
#   packaging/build-manylinux-wheel.sh [output-dir]   (default: dist/)
#
# Needs Docker. The wheel is built for the machine's own architecture
# (x86_64 or aarch64), or the one given by CAMLIB_ARCH if Docker can run it.
# Rust is installed in the container at the version given by CAMLIB_RUST
# (default: the host's rustc version), so the wheel is built with the
# toolchain the tests ran on. auditwheel checks every native file links only
# manylinux_2_28 system libraries and tags the wheel.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
out=$(mkdir -p "${1:-$root/dist}" && cd "${1:-$root/dist}" && pwd)
rust=${CAMLIB_RUST:-$(rustc --version | cut -d' ' -f2)}
arch=${CAMLIB_ARCH:-$(uname -m)}
case "$arch" in
    x86_64 | aarch64) ;;
    *) echo "unsupported architecture $arch: manylinux wheels are built for x86_64 and aarch64" >&2; exit 2 ;;
esac
image=quay.io/pypa/manylinux_2_28_$arch
git -C "$root" archive --format=tar HEAD | docker run --rm -i --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/home -e RUST="$rust" -e ARCH="$arch" -v "$out":/out "$image" bash -euo pipefail -c '
  mkdir -p /tmp/build /tmp/wheel && tar -x -C /tmp/build
  curl -sSf https://sh.rustup.rs | sh -s -- -y -q --profile minimal --default-toolchain "$RUST" >/dev/null
  . "$HOME/.cargo/env"
  /opt/python/cp312-cp312/bin/python -m pip wheel -q --no-deps /tmp/build -w /tmp/wheel
  auditwheel repair --plat "manylinux_2_28_$ARCH" -w /out /tmp/wheel/*.whl
'
ls -l "$out"/*manylinux_2_28*.whl
