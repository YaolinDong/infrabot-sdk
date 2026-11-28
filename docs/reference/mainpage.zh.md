# InfraBot SDK

控制器 UDP 协议的 PC 端，提供 C++ 接口和 Python 绑定：每个机械臂控制器对应一个
`infrabot::Robot`。它管理链路、让控制器保持存活，并在程序停止通信时托住机械臂。

## 从哪开始

程序按下面的顺序调用。每一步都是 \ref infrabot::Robot 中的一个条目。

1. 调用 `connect()`，等到 `state().telemetry_age_ms` 小于 100 ms。
2. 在所有电机失能的状态下调用 `set_motor_profile()`。
3. 调用 `servo_on(MotorGroup::Arm)`；机械臂在 IDLE 下保持。
4. 用 `switch_mode()` 进入程序需要的模式。
5. 运动：NRT 模式下用 `movej()` 和 `movep()`，RT 模式下用 `servej()` 和 `servep()`。
6. 用 `stop()` 制动并保持机械臂，然后 `disconnect()`。

## 接下来看哪里

- \ref infrabot::Robot "Robot 类"：每个调用需要什么前提、失败时如何。
- \ref examples "示例"：完整的 C++ 程序。
- \ref infrabot "infrabot 命名空间"：类型、枚举和自由函数。

使用说明在仓库中：
[API 使用](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/api.md) ·
[实时循环](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/realtime.md) ·
[协议](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/protocol.md) ·
[固件升级](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/firmware-update.md) ·
[更新记录](https://github.com/YaolinDong/infrabot-sdk/blob/main/CHANGELOG.md)

## 时序分类

每个调用都标明它的行为：

- **[Blocking]** 等待控制器，最长是标注的时间。不要在 1 kHz 循环里调用。
- **[Non-blocking]** 立即返回：发出一个数据报或复制状态。
- **[RT]** 不阻塞，成功时不分配内存：可以在 1 kHz 循环里调用。

调用失败时返回 `false`，并把原因（中文）写入 `error`。
