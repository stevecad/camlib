# Releasing

camlib is published on PyPI as **`cam-engine`**; the package it installs is
imported as `camlib`. (PyPI's `camlib` belongs to an unrelated project.)

## What a release publishes

Pushing a tag `v<version>` runs `.github/workflows/release.yml`, which:

1. checks that the tag is the version in `pyproject.toml`, that both Rust
   crates declare the same version, and that `CHANGELOG.md` has a section for
   it (`packaging/check_release.py`);
2. builds a wheel for Linux x86_64 and aarch64 (manylinux_2_28, in the PyPA
   build image) and for Windows x64, and the source distribution;
3. installs each one outside the source tree and runs
   `python/tests/test_package.py` against it: the standalone example must
   write its inspection program, and must refuse export without the
   acceptance flag. The Linux wheels are tested in a clean `python:3.12-slim`
   container with no Rust and nothing of the repository installed;
4. publishes them to PyPI and creates the GitHub release, with notes
   generated from the pull requests merged since the last tag.

Nothing is published if any step before fails. PyPI does not allow a version
to be uploaded twice, so a release that published cannot be repeated: fix
forward with the next patch version.

## Cutting a release

1. In a pull request: set the version in `pyproject.toml`, `Cargo.toml` and
   `bindings/python/Cargo.toml` (run `cargo check` so `Cargo.lock` follows),
   and add a `## <version>` section to `CHANGELOG.md`. Check it with
   `python packaging/check_release.py v<version>`.
2. After it has merged, tag the merge commit on `main` and push the tag:

   ```sh
   git tag v0.1.0 <commit>
   git push origin v0.1.0
   ```

3. Watch the Release workflow. When it finishes, `pip install cam-engine` in a
   clean environment gives the new version.

## How publishing is authorised

PyPI trusted publishing: the PyPI project trusts the workflow
`release.yml` of `stevecad/camlib` running in the GitHub environment `pypi`,
and GitHub proves that identity to PyPI for each run. There is no API token.
The workflow's file name and the environment's name are part of that
registration (PyPI, project settings, Publishing); renaming either stops
publishing until it is changed there too. Required reviewers on the `pypi`
environment (repository Settings, Environments) add an approval before each
publish.

## Platforms

Wheels are built only for platforms the wheel is tested on in the release
workflow. macOS and other platforms install from the source distribution,
which compiles the engine and needs a Rust toolchain (1.88 or newer).
