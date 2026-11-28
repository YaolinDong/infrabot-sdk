/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once

/**
 * @file firmware_update.hpp
 * @brief Signed firmware updates over Ethernet.
 *
 * These calls use their own maintenance socket on port 5002. They do not need a
 * connected Robot and never enable or disable a drive. They are [Blocking]: run
 * them outside the control loop. Support the arm, disable the drives and
 * disconnect motor power before updating.
 */

#include <cstdint>
#include <functional>
#include <string>

namespace infrabot {

/**
 * @brief [Blocking] Makes the board boot the other A/B slot.
 *
 * Use it when a new firmware misbehaves: the board goes back to the image in
 * its other slot. The board must run its normal application; approve the
 * request on the touchscreen. The call returns once the other slot runs.
 * @param ipv4 Controller IPv4 address.
 * @param public_key_file Publisher public key; empty selects the SDK's release key.
 * @param port Maintenance port.
 * @throws std::runtime_error On refusal, timeout or verification failure.
 */
void rollback_firmware(const std::string &ipv4,const std::string &public_key_file = {},uint16_t port=5002);
/**
 * @brief [Blocking] Reports the running slot: A or B, confirmed or on trial, and its image ID.
 *
 * E.g. "confirmed A 3f9c…", or "recovery-loader" while the board waits in its
 * loader. Check it before and after an update to see which image runs.
 * @param ipv4 Controller IPv4 address.
 * @param public_key_file Publisher public key; empty selects the SDK's release key.
 * @param port Maintenance port.
 * @return A human-readable status.
 * @throws std::runtime_error On timeout or verification failure.
 */
std::string firmware_status(const std::string &ipv4,const std::string &public_key_file = {},uint16_t port=5002);

/**
 * @brief [Blocking] Installs a signed IFW package after approval on the touchscreen.
 *
 * The board asks for approval on its touchscreen (60 s), writes the image
 * into its inactive slot and boots it on trial. The call returns true once
 * the new application confirms itself; a trial that fails returns the board
 * to the previous firmware and the call throws. Completion requires a reply
 * from the rebooted application with the expected image tag.
 * @param ipv4 Controller IPv4 address.
 * @param package_path The .ifw package.
 * @param public_key_file Publisher public key; empty selects the SDK's release key.
 * @param progress Called with bytes sent and the total.
 * @param port Maintenance port.
 * @return false without sending anything when the board already runs an image from this package.
 * @throws std::runtime_error On refusal, timeout, an invalid package or a verification failure.
 */
bool update_firmware(const std::string &ipv4, const std::string &package_path,
                     const std::string &public_key_file = {},
                     std::function<void(unsigned,unsigned)> progress = {}, uint16_t port = 5002);
}
