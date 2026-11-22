# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
from pathlib import Path
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

setup(
    packages=["infrabot"],
    py_modules=["infrabot_native"],
    package_dir={"": "python"},
    package_data={"infrabot": ["release.pub", "py.typed", "*.pyi"]},
    ext_modules=[Pybind11Extension(
        "infrabot.infrabot_native",
        ["python/bindings.cpp", "src/firmware_update.cpp", "src/robot.cpp", "src/recorder.cpp", "src/serial_log.cpp", "src/run_log.cpp", "src/diagnostics.cpp",
         "third_party/infrabot_ik/src/kinematics.cpp", "third_party/infrabot_ik/src/ik_solver.cpp"],
        include_dirs=["include", "src", "third_party/infrabot_ik/include"],
        define_macros=[("INFRABOT_RELEASE_PUBLIC", '"'+(Path(__file__).resolve().parent/"firmware/release.pub").read_text().strip()+'"')],
        libraries=["crypto"], cxx_std=17, extra_compile_args=["-O2", "-pthread"], extra_link_args=["-pthread"],
    )],
    cmdclass={"build_ext": build_ext},
)
