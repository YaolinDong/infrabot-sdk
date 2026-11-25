# Controller protocol

The firmware repository owns `protocol/infrabot/protocol.h`. The [SDK copy](../include/infrabot/protocol.h) defines the packed little-endian wire types and size assertions. CRC-protected packets end in a CRC-32 of the preceding bytes.

| UDP port | Purpose |
| --- | --- |
| 5000 | Telemetry registration, DMTL telemetry, maintenance/info replies and clock synchronization |
| 5001 | DMIF info requests, DMCP commands, DMOV/DMOS motion sessions and DMPR/DMPA parameter requests |
| 5002 | Firmware update service |
| 5003 | On-board run log download |

## Discovery and compatibility

SDK 0.1.4 implements protocol 1.0. A 16-byte DMIF query is sent from the host command socket. The MCU sends DMIO from its telemetry port back to that socket. The SDK validates source endpoint, request sequence, length and CRC before starting the heartbeat. V2 (218 bytes) reports the protocol major/minor, firmware build string, board and arm model. `RobotInfo.protocol_version` identifies the DMIO envelope; `protocol_major` and `protocol_minor` identify the overall protocol.

The SDK first queries V2 twice with a 25 ms wait per attempt, then allows 450 ms for V1. A V1-only MCU counts those two unsupported V2 requests as invalid packets and drops them; neither executes a motor command. Its 154-byte V1 reply is interpreted as protocol 1.0 and its firmware string as `unknown`. An old SDK can continue to request V1 directly from current firmware.

A different major or a handshake timeout fails `connect`; C++ returns `false` with an error and Python raises `RuntimeError`. Minor revisions are additive. Select optional features using the received capability mask.

| Mask | `INFRABOT_CAP_` suffix | Feature |
| --- | --- | --- |
| `0x0001` | `JOINT_RT` | RT joint targets |
| `0x0002` | `JOINT_NRT` | NRT joint moves |
| `0x0004` | `MAINTENANCE` | Drive and register maintenance |
| `0x0008` | `PERSISTENCE` | Persistent configuration |
| `0x0010` | `CARTESIAN_NRT` | NRT Cartesian moves |
| `0x0020` | `CARTESIAN_RT` | RT Cartesian pose stream |
| `0x0040` | `GROUPS` | Independent arm and gripper enable groups |
| `0x0080` | `TELEMETRY_V3` | Per-drive feedback and command fields |
| `0x0100` | `CONTROLLED_STOP` | Stop and safety status, brake to hold |
| `0x0200` | `SELFCHECK` | Motor profile parameter self-check |
| `0x0400` | `RESUME` | Resume a controlled stop |
| `0x0800` | `LOCAL_GRAVITY` | On-controller gravity compensation |
| `0x1000` | `ADMITTANCE` | Joint and Cartesian admittance |
| `0x2000` | `RUN_LOG` | SDRAM text and telemetry log download |
| `0x4000` | `PARAMETERS` | Parameter service (DMPR/DMPA), protocol 1.1 |

## Parameter service

Protocol 1.1 adds one request and one reply on the command port, answered to the sender. A DMPR (24 bytes) carries an operation, a request id and a parameter id. A DMPA (80 bytes) echoes the request id and describes the addressed parameter: name, unit, minimum, maximum, default, slew rate, and the requested and applied values. The parameter id is the group in the high byte (1 gravity, 2 impedance, 3 admittance) and the index in the low byte. INFO takes a list position instead and also returns the parameter count, so the PC lists every parameter without a table of its own and a firmware that adds parameters needs no new protocol version.

| Operation | Meaning |
| --- | --- |
| `INFO` | Descriptor of the parameter at a list position |
| `GET` | Descriptor of a parameter by id |
| `SET` | Store a requested value; the reply shows the state after the write |
| `SAVE` | Reserved; answered `UNSUPPORTED` |

A write never steps a gain: the controller slews the applied value toward the requested one at the parameter rate. Values live in RAM and return to their defaults at reboot. INFO and GET are open to every PC. SET needs the PC that controls the arm, or no controlling PC, and is otherwise answered `BUSY`. A value that is not finite or lies outside the range is answered `RANGE` and changes nothing. Firmware without `INFRABOT_CAP_PARAMETERS` does not answer, and the SDK refuses before it sends.

## Verify a checkout

Set `INFRABOT_FIRMWARE_DIR` to the firmware root when configuring the SDK tests. `ctest --test-dir build-tests -R '^shared_protocol$' --output-on-failure` compares the headers byte for byte. If you omit the firmware checkout, CTest reports a skip. Apply coordinated updates with `tools/sync_protocol.sh /path/to/infrabot-firmware`.
