# Changelog

## 0.2.1 — 2026-11-28

- API reference built by Doxygen from the source: four sites per release, English and Chinese, each with the C++ API and the Python API, and every page switches between the two languages and the two APIs (`docs/reference/doxygen_site.py`, CMake target `api_doxygen`). Every public C++ name is documented, and an undocumented one fails the build.
- Every entry of the reference explains what the call is for and how to use it, and the Examples tab shows the programs of `examples/` in full, the `.cpp` ones in the C++ sites and the `.py` ones in the Python sites.
- Python docstrings for every class, field and method, also in the type stub that IDEs read. The ctest `python_docs` fails while a public name lacks one.
- `infrabot/version.hpp` is generated from `project(VERSION)`, so `kSdkVersion` matches the release. The 0.2.0 header still said 0.1.2.

## 0.2.0 — 2026-11-26

- Parameter service (protocol 1.1): `list_parameters`, `get_parameter` and `set_parameter` in C++ and Python read and write the gravity, impedance and admittance settings that were only reachable from the touchscreen. Firmware without `INFRABOT_CAP_PARAMETERS` is refused before anything is sent.
- Example `08_parameters` in C++ and Python.
- Diagnostic snapshot: `diagnostics()` in C++ and Python reads loop timing, CAN and command counters, heap, least free task stacks and drive states (`INFRABOT_CAP_DIAGNOSTICS`, read only, open to any PC). Example `09_diagnostics`.
- Examples 10–17 in C++ and Python: monitor, joint and Cartesian offset moves, RT Cartesian stream, every local compliance mode, controlled stop and resume, gripper, recording. `docs/examples.md` lists them all.
- `switch_mode` waits up to 1 s for the MCU to hand over control when another program on this PC stopped sending less than 0.5 s ago, instead of failing with "请先全部失能", and for the 0.5 s in which a stopped local mode answers BUSY. A controller that keeps sending is reported as such.
- The hold move that opens an NRT session accepts 0.05 rad (0.05 m) of distance, so a stiffness-only (MIT) arm sagging below its command no longer times out on entering `NrtJoint` or `NrtCartesian`; it still has to be still.
- A refused admittance mode says that every joint needs 0.03 rad of margin to its limit (J2 and J4 start at 0).
- Read-only examples no longer send heartbeats. The loopback test runs every Python example and every C++ example program.
- The protocol header is the firmware 1.1 header.

## 0.1.4 — 2026-11-26

- `flash-net --download` without a version finds the latest release again. GitHub now redirects a release download on to a storage host whose URL does not name the release, so the version is read from every redirect, not only the last one.

## 0.1.3 — 2026-11-24

- The updater reports `The board already runs this firmware; nothing was sent.` instead of a transfer and a confirmation when the board already runs an image from the package. `update_firmware` returns whether it installed an image (Python: `bool`).

## 0.1.2 — 2026-11-23

- `flash-net` and `flash-swd` accept `--download [VERSION]` to fetch the signed package from the firmware GitHub Release, verify it and cache it under `firmware/cache/`. `--url` selects a mirror.

## 0.1.1 — 2026-11-23

- The firmware updater sends the new image's version, so firmware 0.1.1 can show it on the update prompt. Firmware 0.1.0 is still updated as before.

## 0.1.0 — 2026-11-23

- C++17 Robot API and Python bindings for H7C joint and Cartesian control, with local gravity, impedance and admittance modes.
- MCU telemetry, recording, serial and on-board log retrieval, plus an optional Qt control GUI with an external H7C model.
- V2 controller discovery with protocol major/minor and firmware identity. V1-only controllers are supported as protocol 1.0.
- CMake package export, Python wheel, signed firmware maintenance tools and paired C++/Python examples.
- Before 1.0 the CMake package matches only the same minor version, because a 0.x minor release may break the API.
