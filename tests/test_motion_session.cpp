/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "motion_session.hpp"
#include <cassert>
#include <iostream>

using namespace motion;
static Parameters parameters() {
    Parameters p;
    p.velocity_max.fill(.1f); p.acceleration_max.fill(.2f);
    p.kp.fill(30); p.kd.fill(.8f);
    return p;
}
static Reply response(const Request &q, uint8_t state, uint32_t elapsed = 1) {
    Reply r{};
    r.magic = INFRABOT_MAGIC_MOVE_REPLY; r.version = INFRABOT_MOVE_V1; r.state = state;
    r.session = q.session; r.request_id = r.active_id = q.motion_id;
    r.elapsed_ms = elapsed; r.duration_ms = 100;
    r.enabled_mask = r.fresh_mask = 0x7f;
    r.arrived_mask = state == Done ? 0x7f : 0;
    r.crc32 = crc32(&r, sizeof(r) - 4);
    return r;
}
static void send_reply(Session &s, const Request &q, uint8_t state, uint64_t now, uint32_t elapsed = 1) {
    auto r = response(q, state, elapsed);
    assert(s.receive(&r, sizeof(r), now));
}
int main() {
    Session s;
    std::string error;
    Request q{};
    auto p = parameters();
    assert(s.prepare(p, 42, 1000, error));
    assert(s.preparing() && !s.poll(2000, q));
    assert(!s.movej(p, 2000, error));
    std::array<float, axes> actual{}; actual.fill(.15f);
    s.begin_hold(actual, 51000);
    assert(s.poll(51000, q) && q.motion_id == 1 && q.op == MoveJ);
    assert(q.command.joint.target[0] == .15f && q.command.joint.kp[0] == 30);
    assert(q.crc32 == crc32(&q, sizeof(q) - 4));
    assert(!s.poll(51500, q));
    Request original = q;
    // Late wakes don't accumulate a permanent slow rate; 100 packets/second nominal.
    int count = 1;
    for (uint64_t now = 52000; now < 251000; now += 1000) {
        if (s.poll(now + 15, q)) {
            ++count;
            assert(std::memcmp(&q, &original, sizeof(q)) == 0);
            send_reply(s, q, Running, now + 16, static_cast<uint32_t>(now));
        }
    }
    assert(count == 20);
    send_reply(s, q, Done, 251001, 251000);
    assert(s.snapshot(251002).phase == Phase::Ready);
    assert(s.poll(261000, q) && q.op == Keepalive);
    const auto *raw = reinterpret_cast<const uint8_t *>(&q.command);
    for (size_t i = 0; i < sizeof(Parameters); ++i) assert(raw[i] == 0);
    p.target.fill(.2f);
    assert(s.movej(p, 261001, error));
    assert(s.poll(261002, q) && q.motion_id == 2);
    auto stale = response(original, Done);
    assert(!s.receive(&stale, sizeof(stale), 261003));
    auto foreign = response(q, Done);
    ++foreign.session; foreign.crc32 = crc32(&foreign, sizeof(foreign) - 4);
    assert(!s.receive(&foreign, sizeof(foreign), 261004));
    auto damaged = response(q, Done); damaged.crc32 ^= 1;
    assert(!s.receive(&damaged, sizeof(damaged), 261004));
    assert(!s.movej(p, 261005, error));
    send_reply(s, q, Settling, 270000, 30);
    assert(!s.snapshot(270001).can_execute);
    send_reply(s, q, Done, 280000, 40);
    assert(s.snapshot(280001).can_execute);
    stale = response(q, Running, 41);
    assert(!s.receive(&stale, sizeof(stale), 280002));
    assert(s.snapshot(280003).phase == Phase::Done);
    // A cancel is a DMOV cancel only: the controller stops the arm in a
    // controlled way and holds it; the session never asks for DISABLE.
    s.cancel(290000);
    assert(s.poll(290001, q) && q.op == Cancel);
    send_reply(s, q, Cancelled, 290002);
    assert(s.snapshot(290003).phase == Phase::Cancelled && !s.engaged());
    assert(!s.poll(300000, q));
    assert(s.prepare(p, 43, 310000, error));
    s.begin_hold(actual, 310001);
    s.poll(310002, q);
    // No DMOS for 300 ms: the session fails and cancels (no DISABLE).
    assert(s.poll(620002, q) && q.op == Cancel);
    assert(s.snapshot(620003).phase == Phase::Error);
    send_reply(s, q, Cancelled, 620004); // Does not erase original timeout.
    assert(s.snapshot(620005).phase == Phase::Error);
    s.poll(880000, q);
    assert(s.prepare(p, 44, 890000, error));
    s.begin_hold(actual, 900000); s.poll(900001, q);
    auto failure = response(q, Failed); failure.reason = 10;
    failure.crc32 = crc32(&failure, sizeof(failure) - 4);
    assert(s.receive(&failure, sizeof(failure), 910000));
    assert(s.snapshot(910001).error.find("MOTOR_FAULT") != std::string::npos);
    assert(s.poll(910001, q) && q.op == Cancel); // stopped, not disabled
    assert(!s.movej(p, 910002, error));
    p.kd[0] = 6;
    assert(!valid(p, error));
    std::cout << "native motion session: rate, hold, completion, busy, CRC/session, cancel, timeouts passed\n";
}
