/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* protocol.h as the firmware compiles it (C11): CRC, offsets, register table. */
#include "infrabot/protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t bits(float value)
{
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  return raw;
}

int main(void)
{
  for(unsigned mode=0;mode<=10;mode++)for(unsigned profile=1;profile<=2;profile++)
  for(unsigned flags=0;flags<4;flags++){
    const uint8_t packed=(uint8_t)(mode|(profile<<4)|(flags<<6));
    assert(INFRABOT_MODE_FROM_BYTE(packed)==mode);
    assert(INFRABOT_PROFILE_FROM_BYTE(packed)==profile);
  }
  assert(infrabot_crc32("123456789", 9U) == 0xCBF43926U);
  /* Fields the two sides have relied on since v2/v3. */
  assert(offsetof(InfrabotCommand, target_position_mrad) == 32U);
  assert(offsetof(InfrabotMaintenance, register_value) == 92U);
  assert(offsetof(InfrabotJointStream, velocity_limit_mrad_s) == 192U);
  assert(offsetof(InfrabotCartesianStream, cartesian_limits) == 272U);
  assert(offsetof(InfrabotTelemetry, command_last_sequence) == 152U);
  assert(offsetof(InfrabotTelemetry, status) == 161U && offsetof(InfrabotTelemetry, mode) == 163U);
  assert(offsetof(InfrabotTelemetryV3, command) == 180U);
  assert(offsetof(InfrabotTelemetryV3, link) == 308U && offsetof(InfrabotTelemetryV3, safety) == 328U);
  assert(offsetof(InfrabotMoveJoint, torque) == 156U && offsetof(InfrabotMoveCartesian, cartesian_limits) == 176U);
  /* Register table: every entry valid, ids unique, the firmware's types. */
  unsigned count = 0U;
  for (unsigned rid = 0U; rid < 256U; ++rid)
  {
    const InfrabotRegisterInfo *info = infrabot_register_info((uint8_t)rid);
    if (info == NULL) continue;
    ++count;
    assert(info->rid == rid && info->name && info->description);
    const int u32 = (rid >= 7U && rid <= 10U) || (rid >= 13U && rid <= 16U) || rid == 35U || rid == 36U;
    assert(info->type == (u32 ? INFRABOT_REGISTER_U32 : INFRABOT_REGISTER_F32));
    if (info->access != INFRABOT_REGISTER_READ_ONLY) assert(info->low <= info->high);
  }
  assert(count == 53U);
  assert(infrabot_register_write_allowed(0U, bits(15.0f)) && !infrabot_register_write_allowed(0U, bits(10.0f)));
  assert(infrabot_register_write_allowed(2U, bits(80.0f)) && !infrabot_register_write_allowed(2U, bits(200.0f)));
  assert(!infrabot_register_write_allowed(3U, bits(0.0f)) && !infrabot_register_write_allowed(3U, bits(1.0f)));
  assert(infrabot_register_write_allowed(5U, bits(-50.0f)) && !infrabot_register_write_allowed(5U, bits(0.0f)));
  assert(!infrabot_register_write_allowed(INFRABOT_RID_MST_ID, 0x11U) && infrabot_register_value_valid(INFRABOT_RID_MST_ID, 0x11U));
  assert(!infrabot_register_write_allowed(INFRABOT_RID_CAN_BR, 9U));
  assert(infrabot_register_write_allowed(INFRABOT_RID_TIMEOUT, 20U) && !infrabot_register_write_allowed(INFRABOT_RID_TIMEOUT, 19U));
  assert(infrabot_register_write_allowed(INFRABOT_RID_TIMEOUT, 0xFFFFFFFFU));
  assert(infrabot_register_write_allowed(INFRABOT_RID_CTRL_MODE, 4U) && !infrabot_register_write_allowed(INFRABOT_RID_CTRL_MODE, 0U));
  assert(infrabot_register_write_allowed(INFRABOT_RID_VMAX, bits(10.0f)) && !infrabot_register_write_allowed(INFRABOT_RID_VMAX, bits(0.0f)));
  assert(!infrabot_register_write_allowed(25U, 0x7FC00000U)); /* NaN */
  assert(!infrabot_register_write_allowed(80U, 0U));          /* read-only */
  assert(INFRABOT_SELFCHECK_REPLY_VALUE(2U, INFRABOT_RID_VMAX, INFRABOT_SELFCHECK_MISMATCH) == 0x021602U);
  puts("protocol.h (C11): sizes, offsets, CRC, register table passed");
  return 0;
}
