# Changelog

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
