# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
import re
from pathlib import Path
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ROOT = Path(__file__).resolve().parent
VERSION = re.search(r"project\(infrabot_sdk VERSION (\d+\.\d+\.\d+)",
                    (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")).group(1)

# infrabot/version.hpp carries project(VERSION). CMake generates it from
# cmake/version.hpp.in for the CMake build; this makes the same file for the
# Python one, which compiles src/ directly and never runs CMake.
GENERATED = ROOT / "build" / "generated" / "include"
header = GENERATED / "infrabot" / "version.hpp"
header.parent.mkdir(parents=True, exist_ok=True)
major, minor, patch = VERSION.split(".")
header.write_text((ROOT / "cmake" / "version.hpp.in").read_text(encoding="utf-8")
                  .replace("@PROJECT_VERSION_MAJOR@", major)
                  .replace("@PROJECT_VERSION_MINOR@", minor)
                  .replace("@PROJECT_VERSION_PATCH@", patch)
                  .replace("@PROJECT_VERSION@", VERSION), encoding="utf-8")

setup(
    packages=["infrabot"],
    py_modules=["infrabot_native"],
    package_dir={"": "python"},
    package_data={"infrabot": ["release.pub", "py.typed", "*.pyi"]},
    ext_modules=[Pybind11Extension(
        "infrabot.infrabot_native",
        ["python/bindings.cpp", "src/firmware_update.cpp", "src/robot.cpp", "src/recorder.cpp", "src/serial_log.cpp", "src/run_log.cpp", "src/diagnostics.cpp",
         "third_party/infrabot_ik/src/kinematics.cpp", "third_party/infrabot_ik/src/ik_solver.cpp"],
        include_dirs=[str(GENERATED), "include", "src", "third_party/infrabot_ik/include"],
        define_macros=[("INFRABOT_RELEASE_PUBLIC", '"'+(Path(__file__).resolve().parent/"firmware/release.pub").read_text().strip()+'"')],
        libraries=["crypto"], cxx_std=17, extra_compile_args=["-O2", "-pthread"], extra_link_args=["-pthread"],
    )],
    cmdclass={"build_ext": build_ext},
)
