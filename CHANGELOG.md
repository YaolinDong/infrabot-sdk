# Changelog

## 0.1.0 — unreleased

- C++17 Robot API and Python bindings for H7C joint and Cartesian control, with local gravity, impedance and admittance modes.
- MCU telemetry, recording, serial and on-board log retrieval, plus an optional Qt control GUI with an external H7C model.
- V2 controller discovery with protocol major/minor and firmware identity. V1-only controllers are supported as protocol 1.0.
- CMake package export, Python wheel, signed firmware maintenance tools and paired C++/Python examples.
- Before 1.0 the CMake package matches only the same minor version, because a 0.x minor release may break the API.
