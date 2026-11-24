/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <cstdint>
#include <functional>
#include <string>

namespace infrabot {
/* [Blocking] Maintenance calls wait for network replies; run outside the control loop. */
void rollback_firmware(const std::string &ipv4,const std::string &public_key_file = {},uint16_t port=5002);
std::string firmware_status(const std::string &ipv4,const std::string &public_key_file = {},uint16_t port=5002);

// Dedicated maintenance socket; does not connect a Robot, enable or disable motors.
// Throws std::runtime_error on refusal, timeout, invalid package, or verification failure.
// Completion requires a reply from the rebooted application with the expected image tag.
// Returns false without sending anything when the board already runs an image from this package.
bool update_firmware(const std::string &ipv4, const std::string &package_path,
                     const std::string &public_key_file = {},
                     std::function<void(unsigned,unsigned)> progress = {}, uint16_t port = 5002);
}
