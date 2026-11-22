# InfraBot IK

The C++ headers and solver sources come from infrabot_ik commit `b5d4263` (2026-10-04). The compiled H7C geometry lives in `include/infrabot_ik/generated_model.hpp`. SDK forward kinematics works without model files, Qt or ROS.

The viewer from the same checkout, including the local `setHomeView()` addition, lives in `tools/gui/`. Set `INFRABOT_DESCRIPTION_DIR` to the package root containing `urdf/` and `meshes/`. [H7C model](https://github.com/YaolinDong/infrabot-description)

After changing geometry, run `tools/export_kinematics.py <URDF> <SDK>/third_party/infrabot_ik/include/infrabot_ik/generated_model.hpp --sdk` from the firmware checkout, then `tests/test_h7c_kinematics.py --sdk <SDK>` to compare both compiled solvers with the firmware URDF.

License: [Apache-2.0](LICENSE).
