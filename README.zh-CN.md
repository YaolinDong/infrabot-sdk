# InfraBot SDK

[English](README.md) · 简体中文

面向 Linux 的 C++17 和 Python SDK，支持 InfraBot H7C 七轴机械臂和可选 J8 夹爪。一个 `Robot` 对象负责 UDP 握手、读取遥测和发送运动指令。Qt 控制界面也使用这套接口。

## 安装

在 SDK 源码目录操作。Python 需要 3.10 或更高版本，以下命令使用已激活的虚拟环境。

```bash
sudo apt install build-essential cmake libssl-dev python3-dev python3-venv
python -m pip install .
```

C++ 应用先安装 CMake 包：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DINFRABOT_BUILD_GUI=OFF
cmake --build build --parallel
cmake --install build --prefix "$HOME/.local"
```

```cmake
find_package(infrabot_sdk CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE infrabot_sdk::infrabot_sdk)
```

配置调用方工程时传 `-DCMAKE_PREFIX_PATH="$HOME/.local"`。需要 GUI 时安装 `qtbase5-dev libqt5opengl5-dev`，配置 `-DINFRABOT_BUILD_GUI=ON`。将 `INFRABOT_DESCRIPTION_DIR` 指向 [H7C 描述包](https://github.com/YaolinDong/infrabot-description) 的根目录即可显示实测三维位姿。

## 连接并读取关节状态

电脑与控制板应在同一子网。下面的示例关闭心跳，读取固件版本，并等待最多半秒取得 J1 角度（弧度）。控制板会将遥测发往注册者，使用示例前先关闭当前控制客户端。

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
    std::cout << "固件 " << robot.info().firmware << '\n';
    for (int i = 0; i < 50; ++i) {
        const auto s = robot.state();
        if (s.joints.host_time_us) {
            std::cout << "J1 " << s.joints.position[0] << " rad\n";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "未收到遥测\n";
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
    print("固件", robot.info().firmware)
    for _ in range(50):
        state = robot.state()
        if state.joints.host_time_us:
            print("J1", state.joints.position[0], "rad")
            break
        time.sleep(0.01)
    else:
        raise TimeoutError("未收到遥测")
finally:
    robot.disconnect()
```

## 控制模式

| 模式 | 执行位置 | PC 在运行期间的发送要求 |
| --- | --- | --- |
| Idle | MCU 为已使能的电机保持位置 | SDK 默认发送 100 Hz 心跳 |
| RtJoint | 实时关节目标，MCU 以 1 kHz 驱动电机 | 通常以 1 kHz 调 `servej`；也可用 100–1000 Hz 原生发送线程，并持续更新目标 |
| RtCartesian | 实时末端位姿，MCU 执行笛卡尔跟踪 | 通常以 1 kHz 调 `servep` |
| NrtJoint | MCU 规划并执行 `movej` | 提交目标后，SDK 以 100 Hz 维持运动会话 |
| NrtCartesian | MCU 规划并执行 `movep` | 提交目标后，SDK 以 100 Hz 维持运动会话 |
| GravityFree、GravityLock | MCU 本地 1 kHz 重力补偿 | 进入模式后无需 PC 目标流 |
| JointImpedance、CartesianImpedance | MCU 本地 1 kHz 阻抗控制 | 进入模式后无需 PC 目标流 |
| JointAdmittance、CartesianAdmittance | MCU 本地 1 kHz 导纳控制 | 进入模式后无需 PC 目标流 |

**安全。** 使用独立硬件急停，使能前核对运动空间、零位和驱动方向。当前固件在依赖通信的模式下遇到断流会受控停止，通常保持使能并回到 IDLE。本地重力、阻抗和导纳模式可在 PC 断开后继续运行。切断电机动力前支撑好机械臂。1 kHz 循环只调用标为 `[RT]` 的接口。[接口时序与停止语义](docs/api.md) · [实时循环](docs/realtime.md)

## 兼容性与文档

SDK 0.2.0 使用协议 1.1，也能连接协议 1.0 的固件；参数服务和诊断查询需要固件 0.2.0。V2 握手可以读取固件、板型和机械臂型号。仅支持 V1 的固件按 1.0 连接，固件版本显示 `unknown`。主版本不同或握手超时会在启动心跳前返回错误；可选功能按能力位判断。

[协议与能力位](docs/protocol.md) · [固件升级](docs/firmware-update.md) · [C++ 示例](examples/cpp) · [Python 示例](examples/python) · [构建与测试](CONTRIBUTING.md)

[Apache-2.0](LICENSE) · [第三方声明](THIRD_PARTY.md) · [更新记录](CHANGELOG.md) · [安全问题报告](SECURITY.md)
