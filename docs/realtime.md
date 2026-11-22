# A 1 kHz host command stream

In `RtJoint` you can send one joint target per `servej` call, or use the SDK's native periodic sender. In `RtCartesian` a host producer calls `servep` at the desired cadence. The preferred command period for both is 1 ms. The current MCU supervises a 100 ms gap in a required stream; allow margin for scheduling delays. [Paired streaming example](../examples/cpp/04_rt_joint_stream_1khz.cpp)

## Native joint sender

First enter RT Joint with the MIT profile, J1–J7 enabled and fresh v3 feedback. Initialize `JointCommand.position` from the first seven elements of `hold_command()`. `start_joint_stream` starts periodic transmission at `transmit_hz` (default 1000, range 100–1000). Subsequent `servej` calls update the stored target and refresh the producer watchdog. The supplied Python example updates a held target at 100 Hz while the native thread sends it at 1 kHz.

`command_timeout_ms` and `feedback_timeout_ms` both default to 100. Stale commands, stale feedback, a restarted telemetry source or a drive fault can stop the pump and request a controlled stop. Inspect `joint_stream_stats().fault` before restarting. `stop_joint_stream()` ends the native sender; `stop()` requests a controlled stop of the arm. [Python example](../examples/python/04_rt_joint_stream_1khz.py)

## Linux scheduling

Use a PREEMPT_RT kernel when you need to reduce host wake-up latency. Boot the intended kernel and check the running system (`uname -r`; `/sys/kernel/realtime` reports `1` on kernels exporting that attribute). Pick an online CPU in your allowed affinity mask and check the FIFO limit using `ulimit -r`. On a host configured for RT scheduling, the following launches the example pinned to CPU 3 at FIFO priority 50. It enables the arm: clear its workspace and verify the independent emergency stop first.

```bash
taskset -c 3 chrt -f 50 ./build/04_rt_joint_stream_1khz 192.168.0.50
```

The process and its new threads inherit that scheduling context. In an application with dedicated control, receive and logging workers, set affinity/priority per worker and leave CPU time for the network stack and disk writer. The numbered SDK examples use positional IP arguments and ordinary Linux scheduling tools.

## Check the result

- Compare `joint_stream_stats().sent` and `skipped_deadlines` over a measured interval. The pump advances past missed deadlines and avoids sending a burst of old targets.
- Record `state().telemetry_age_ms` and `link_stats().rx_interval_max_us`. When `mcu_valid` is set, also inspect controller command intervals, jitter, missed control ticks and CAN errors.
- Use a monotonic clock with absolute deadlines for a custom producer. Keep file I/O, console output, Python callbacks, firmware maintenance and blocking SDK calls outside the periodic path. Measure under the CPU and network load you intend to use.
