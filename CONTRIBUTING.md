# Contributing

Include the SDK revision, firmware version and the command that reproduces the issue. For changes to motion or protocol behavior, include a loopback test and specify the affected control modes and firmware capabilities. For GUI changes, describe the model and the panel or widget involved.

## Offline tests

From a checkout with a C/C++ compiler, OpenSSL development files, Qt development files and an active Python environment:

```bash
python -m pip install "pybind11>=3" numpy
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Debug \
  -DINFRABOT_BUILD_TESTS=ON -DINFRABOT_BUILD_PYTHON=ON -DINFRABOT_BUILD_EXAMPLES=ON \
  -DPython3_EXECUTABLE="$(command -v python)" -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build-tests --parallel 4
ctest --test-dir build-tests --output-on-failure
```

Set `INFRABOT_FIRMWARE_DIR` to a firmware checkout before configuring to enable canonical header comparison and the firmware FK check. Set `INFRABOT_DESCRIPTION_DIR` to an infrabot-description checkout to exercise the populated 3D view. Qt tests run offscreen; the second Qt test supplies a missing model. To check an asynchronous GUI change, run each test separately with `ctest --test-dir build-tests -R '^qt_motion_loopback$' --repeat until-fail:50 --output-on-failure` and the equivalent command for `qt_without_description`.

The canonical protocol header lives in the firmware repository. Use `tools/sync_protocol.sh /path/to/infrabot-firmware` after a coordinated change. Keep its SDK license notice in the adjacent `.license` file. Regenerate Python type stubs with pybind11-stubgen against the built extension when the bindings change. Keep the two SPDX/copyright header lines and strip trailing whitespace from the generated stub.

## Check an update kit

Supply a signed IFW and its matching public key. Run from the SDK root on Linux:

```bash
python3 -B tools/update/package_update.py --package /path/to/application.ifw --public-key /path/to/release.pub
python3 -B tests/test_update_kit.py /path/to/generated-kit.tar.gz
```

Use the archive path printed by the packager. The test verifies the archive allowlist, checksums, signature rejection and compatibility with the previous filename. Its network probe uses loopback.

Use English commit subjects of the form `type(scope): description`. Keep upstream notices with vendored code and record dependency changes in [THIRD_PARTY.md](THIRD_PARTY.md). Send security reports through the [private reporting channel](SECURITY.md).

## Checking a source release

Set `INFRABOT_PRIVATE_DENYLIST` to a regular file outside the checkout, readable only by its owner (mode 0600), with one literal to reject per line. Run `tools/prepublish_check.sh --history` on the release repository to check the index, all reachable snapshots and Git metadata. In CI without the local file, the checker prints a skip notice for private literals and still checks key markers, paths, licenses and size exceptions.
