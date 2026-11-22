# InfraBot SDK

English · [简体中文](README.zh-CN.md)

Linux C++17 and Python SDK for the InfraBot H7C seven-axis arm and optional J8 gripper. One `Robot` object handles UDP discovery, telemetry and motion commands. The Qt GUI uses the same SDK.

## Install

From the SDK checkout on Ubuntu, install a compiler, Python development headers, CMake and OpenSSL 3 development files. Use an active Python 3.10+ virtual environment.

```bash
sudo apt install build-essential cmake libssl-dev python3-dev python3-venv
python -m pip install .
```

For a native application, build and install the CMake package:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DINFRABOT_BUILD_GUI=OFF
cmake --build build --parallel
cmake --install build --prefix "$HOME/.local"
```

```cmake
find_package(infrabot_sdk CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE infrabot_sdk::infrabot_sdk)
```

Pass `-DCMAKE_PREFIX_PATH="$HOME/.local"` when configuring the consuming project. To build the GUI, install `qtbase5-dev libqt5opengl5-dev` and configure with `-DINFRABOT_BUILD_GUI=ON`. For the measured 3D view, point `INFRABOT_DESCRIPTION_DIR` at the [H7C description](https://github.com/YaolinDong/infrabot-description) checkout.

## Read controller state

Connect the host and controller on the same subnet. The examples query controller information, then wait up to half a second for a joint sample. Joint positions are in radians. Heartbeats are disabled for this read-only session. A telemetry registration selects the receiver on the controller, so close any other active control client first.

```cpp
#include <infrabot/robot.hpp>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    infrabot::Robot robot("192.168.0.50");
    robot.set_heartbeat(false);
    std::string error;
    if (!robot.connect(error)) { std::cerr << error << '\n'; return 1; }
    std::cout << "Firmware " << robot.info().firmware << '\n';
    for (int i = 0; i < 50; ++i) {
        const auto s = robot.state();
        if (s.joints.host_time_us) {
            std::cout << "J1 " << s.joints.position[0] << " rad\n";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "No telemetry sample\n";
    return 1;
}
```

```python
import time
import infrabot

robot = infrabot.Robot("192.168.0.50")
robot.set_heartbeat(False)
robot.connect()
try:
    print("Firmware", robot.info().firmware)
    for _ in range(50):
        state = robot.state()
        if state.joints.host_time_us:
            print("J1", state.joints.position[0], "rad")
            break
        time.sleep(0.01)
    else:
        raise TimeoutError("No telemetry sample")
finally:
    robot.disconnect()
```

## Control modes

| Mode | Where the loop runs | Host traffic while active |
| --- | --- | --- |
| Idle | MCU holds enabled drives | SDK heartbeat at 100 Hz by default |
| RtJoint | RT joint targets; MCU drive loop at 1 kHz | Normally 1 kHz `servej`, or the native joint pump at 100–1000 Hz with fresh producer updates |
| RtCartesian | RT tool pose; MCU runs the Cartesian tracker | Normally 1 kHz `servep` |
| NrtJoint | MCU plans and executes `movej` | Submit a goal; SDK maintains the motion session at 100 Hz |
| NrtCartesian | MCU plans and executes `movep` | Submit a goal; SDK maintains the motion session at 100 Hz |
| GravityFree, GravityLock | MCU local 1 kHz gravity compensation | Mode entry, then no host target stream |
| JointImpedance, CartesianImpedance | MCU local 1 kHz restoring force/torque | Mode entry, then no host target stream |
| JointAdmittance, CartesianAdmittance | MCU local 1 kHz compliant reference | Mode entry, then no host target stream |

**Safety.** Provide an independent hardware emergency stop. Verify clearance, calibration and drive directions before enabling. For current firmware, loss of a required control stream triggers a controlled stop and normally leaves the arm enabled in IDLE hold. Local gravity, impedance and admittance modes can continue after the host disconnects. Support the arm before removing drive power. Keep the 1 kHz path to APIs marked `[RT]`. [API timing and stop behavior](docs/api.md) · [Running the real-time loop](docs/realtime.md)

## Compatibility and guides

Runtime diagnostics and error messages may be in Chinese.

SDK 0.1.0 speaks protocol 1.0. V2 discovery reports firmware, board and model. A V1-only controller is treated as protocol 1.0 with unknown firmware. A different major or an unanswered handshake fails before the heartbeat starts. Use capability bits for optional features.

[Protocol and capabilities](docs/protocol.md) · [Firmware updates](docs/firmware-update.md) · [C++ examples](examples/cpp) · [Python examples](examples/python) · [Build and test](CONTRIBUTING.md)

[Apache-2.0](LICENSE) · [Third-party notices](THIRD_PARTY.md) · [Changelog](CHANGELOG.md) · [Security reporting](SECURITY.md)
