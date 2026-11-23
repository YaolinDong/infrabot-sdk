/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include "infrabot/protocol.h"
#define INFRABOT_SDK_VERSION_MAJOR 0
#define INFRABOT_SDK_VERSION_MINOR 1
#define INFRABOT_SDK_VERSION_PATCH 1

namespace infrabot {
inline constexpr char kSdkVersion[] = "0.1.1";
inline constexpr uint16_t kProtocolMajor = INFRABOT_PROTOCOL_MAJOR;
inline constexpr uint16_t kProtocolMinor = INFRABOT_PROTOCOL_MINOR;
}  // namespace infrabot
