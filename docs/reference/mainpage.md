# InfraBot SDK

The PC side of the InfraBot controller's UDP protocol, in C++ with Python bindings:
one `infrabot::Robot` per arm controller. It owns the link, keeps the controller
alive and holds the arm when the program stops talking.

## Getting started

A program calls these in order. Every step is an entry of \ref infrabot::Robot.

1. `connect()` and wait until `state().telemetry_age_ms` is under 100 ms.
2. `set_motor_profile()` with every drive disabled.
3. `servo_on(MotorGroup::Arm)`; the arm holds in IDLE.
4. `switch_mode()` into the mode the program needs.
5. Move: `movej()` and `movep()` in the NRT modes, `servej()` and `servep()` in the RT modes.
6. `stop()` brakes and holds the arm, then `disconnect()`.

## Where to go

- \ref infrabot::Robot "The Robot class": every call, what it needs and how it fails.
- \ref examples "Examples": complete C++ programs.
- \ref infrabot "The infrabot namespace": the types, the enums and the free functions.

The guides are in the repository:
[Using the API](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/api.md) ·
[Real-time loop](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/realtime.md) ·
[Protocol](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/protocol.md) ·
[Firmware update](https://github.com/YaolinDong/infrabot-sdk/blob/main/docs/firmware-update.md) ·
[Changes](https://github.com/YaolinDong/infrabot-sdk/blob/main/CHANGELOG.md)

## Timing classes

Every call is tagged with how it behaves:

- **[Blocking]** waits for the controller, at most the time given. Never call it from a 1 kHz loop.
- **[Non-blocking]** returns at once: it sends one datagram or copies state.
- **[RT]** non-blocking and allocation-free on success: safe in a 1 kHz loop.

A call that fails returns `false` and writes the reason to `error`, in Chinese.
