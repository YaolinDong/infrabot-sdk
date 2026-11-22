/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
// Deterministic tests of the private pump: deadlines, cancellation, recovery.
#include "joint_stream.hpp"
#include <cassert>
#include <iostream>
using namespace infrabot;
int main() {
    detail::JointStream stream;
    RobotInfo info; info.received = true;
    info.capabilities = INFRABOT_CAP_JOINT_RT | INFRABOT_CAP_TELEMETRY_V3 | INFRABOT_CAP_CONTROLLED_STOP;
    info.velocity_max.fill(30); info.torque_max.fill(50);
    RobotState s; s.connected = true; s.telemetry_age_ms = 0; s.telemetry_version = 3;
    s.mode = Mode::RtJoint; s.profile = MotorProfile::Mit; s.profile_ready = true; s.enabled_mask = 127;
    s.joints.timestamp_us = 1000; s.joints.sequence = 1;
    uint64_t now = 1000; unsigned sent = 0, stops = 0;
    auto send = [&](const JointCommand&,std::string&) { ++sent; return true; };
    auto tick = [&] { stream.tick([&]{return s;},[&]{return now;},send,[&](const char*){++stops;}); };
    JointCommand c; JointStreamOptions o; std::string e;
    assert(stream.serve(c,now,send,e) && sent == 1); // unchanged manual mode
    assert(stream.start(c,o,s,info,now,e));
    sent = 0;
    for (unsigned i=0;i<500;++i) {
        now = 1000 + uint64_t(i)*1000;
        s.joints.timestamp_us = now; ++s.joints.sequence;
        if (i%10 == 0) assert(stream.serve(c,now,send,e));
        tick();
    }
    assert(sent == 500 && stops == 0);
    const auto before = sent;
    now += 20000; s.joints.timestamp_us = now; ++s.joints.sequence;
    assert(stream.serve(c,now,send,e)); tick(); tick();
    assert(sent == before + 1 && stream.stats().skipped_deadlines == 19);
    // Updating after expiration cannot erase an expired producer watchdog.
    now += 101000; s.joints.timestamp_us = now; ++s.joints.sequence;
    assert(!stream.serve(c,now,send,e)); tick();
    assert(stops == 1 && !stream.stats().active);
    assert(!stream.serve(c,now,send,e)); // latched, not implicit direct sending
    assert(stream.start(c,o,s,info,now,e));
    stream.halt(); tick();
    assert(!stream.serve(c,now,send,e));
    stream.release(); assert(stream.serve(c,now,send,e));
    assert(stream.start(c,o,s,info,now,e));
    ++s.telemetry_epoch; tick();
    assert(stops == 2 && !stream.stats().active);
    assert(stream.start(c,o,s,info,now,e));
    for (unsigned i=0;i<110;++i) {
        now += 1000;
        if (stream.stats().active) assert(stream.serve(c,now,send,e));
        tick(); // packets/MCU timestamp frozen despite new commands
    }
    assert(stops == 3 && !stream.stats().active);
    std::cout << "native joint pump cadence, expiry, no catch-up, cancellation and source epoch passed\n";
}
