/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* The UDP protocol between the PC (infrabot SDK, udp_control_gui, the test
 * tools) and the STM32H743 arm controller (infrabot-firmware).  It compiles as C11
 * (the firmware) and C++17 (the PC code).
 *
 * Canonical source: infrabot-firmware protocol/infrabot/protocol.h.
 * The SDK header is a copy of infrabot-firmware protocol/infrabot/protocol.h;
 * do not edit the SDK copy. Use tools/sync_protocol.sh in the SDK.
 *
 * All fields are little endian, floats IEEE754 binary32.  A datagram with a
 * crc32 field ends with the CRC-32 (zlib/Ethernet polynomial, see
 * infrabot_crc32) of all the bytes before it.  Joint values are in the joint
 * frame (the MCU applies each drive's direction and zero offset). */
#ifndef INFRABOT_PROTOCOL_H
#define INFRABOT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#define INFRABOT_STATIC_ASSERT(expression, message) static_assert(expression, message)
#else
#define INFRABOT_STATIC_ASSERT(expression, message) _Static_assert(expression, message)
#endif
#define INFRABOT_PACKED __attribute__((packed))

/* ---- Endpoints and arm layout -------------------------------------------- */
/* MCU telemetry port: sends DMTL/DMAR/DMIO; any datagram from a PC socket to
 * it (other than DMCS) registers that socket as the telemetry destination. */
#define INFRABOT_TELEMETRY_PORT 5000U
/* MCU command port: receives DMCP/DMOV/DMIF, answers DMOV with DMOS. */
#define INFRABOT_COMMAND_PORT 5001U
/* MCU run log port: answers each DMLG request with one DMLR, never sends on
 * its own (5002 is the firmware update service). */
#define INFRABOT_LOG_PORT 5003U
#define INFRABOT_MOTOR_COUNT 8U
/* Application command/trajectory cap for J1-J7, not the motor MIT VMAX
 * encoding range. J8 retains its separate gripper limits. */
#define INFRABOT_ARM_MAX_JOINT_VELOCITY 3.14f
#define INFRABOT_ARM_AXES 7U       /* J1-J7 */
#define INFRABOT_GRIPPER_INDEX 7U  /* J8 */
#define INFRABOT_ARM_MASK 0x7FU
#define INFRABOT_GRIPPER_MASK 0x80U
#define INFRABOT_ALL_MASK 0xFFU
#define INFRABOT_MOTOR_ALL 0xFFU   /* DMCP v4 motor_index: every motor */
/* Telemetry, hold commands and DMAR positions: int32 milliradians. */
#define INFRABOT_NO_JOINT 0xFFU

/* ---- Magics and versions -------------------------------------------------- */
#define INFRABOT_MAGIC_COMMAND 0x444D4350U      /* "DMCP" PC -> MCU */
#define INFRABOT_MAGIC_TELEMETRY 0x444D544CU    /* "DMTL" MCU -> PC, 1 kHz */
#define INFRABOT_MAGIC_MAINTENANCE 0x444D4152U  /* "DMAR" reply to DMCP v4 / v5 store */
#define INFRABOT_MAGIC_INFO_REQUEST 0x444D4946U /* "DMIF" */
#define INFRABOT_MAGIC_INFO_REPLY 0x444D494FU   /* "DMIO" */
#define INFRABOT_MAGIC_SYNC_REQUEST 0x444D4353U /* "DMCS" clock sync */
#define INFRABOT_MAGIC_SYNC_REPLY 0x444D4352U   /* "DMCR" */
#define INFRABOT_MAGIC_MOVE_REQUEST 0x444D4F56U /* "DMOV" movej/movep session */
#define INFRABOT_MAGIC_MOVE_REPLY 0x444D4F53U   /* "DMOS" */
#define INFRABOT_MAGIC_LOG_REQUEST 0x444D4C47U  /* "DMLG" run log request */
#define INFRABOT_MAGIC_LOG_REPLY 0x444D4C52U    /* "DMLR" */
#define INFRABOT_MAGIC_PARAM_REQUEST 0x444D5052U /* "DMPR" parameter service request */
#define INFRABOT_MAGIC_PARAM_REPLY 0x444D5041U   /* "DMPA" parameter service reply */
#define INFRABOT_MAGIC_DIAG_REQUEST 0x444D4452U  /* "DMDR" diagnostic snapshot request */
#define INFRABOT_MAGIC_DIAG_REPLY 0x444D4441U    /* "DMDA" diagnostic snapshot reply */
#define INFRABOT_PARAM_V1 1U
#define INFRABOT_DIAG_V1 1U

#define INFRABOT_COMMAND_V2 2U /* legacy position packet (InfrabotCommandV2) */
#define INFRABOT_COMMAND_V3 3U /* control command / heartbeat (InfrabotCommand) */
#define INFRABOT_COMMAND_V4 4U /* one maintenance operation (InfrabotMaintenance) */
#define INFRABOT_COMMAND_V5 5U /* RT joint stream, profile store (InfrabotJointStream) */
#define INFRABOT_COMMAND_V7 7U /* RT Cartesian stream (InfrabotCartesianStream) */
#define INFRABOT_TELEMETRY_V2 2U /* InfrabotTelemetry, 180 bytes */
#define INFRABOT_TELEMETRY_V3 3U /* InfrabotTelemetryV3, 348 bytes */
#define INFRABOT_MAINTENANCE_V1 1U
/* A major change is incompatible. Minor revisions are additive; use
 * capability bits to discover features. A V1-only reply implies 1.0. */
#define INFRABOT_PROTOCOL_MAJOR 1U
#define INFRABOT_PROTOCOL_MINOR 1U
#define INFRABOT_INFO_V1 1U
#define INFRABOT_INFO_V2 2U
#define INFRABOT_SYNC_V1 1U
#define INFRABOT_MOVE_V1 1U

/* ---- DMCP command codes ------------------------------------------------------
 * v3 carries MIT (stream sample), ENABLE, DISABLE, SET_ZERO, SET_ZERO_ENABLE,
 * STOP, HOLD_HEARTBEAT and the group commands; v4 the maintenance operations;
 * v5 MIT (stream) and STORE_CONTROL_PROFILE; v7 MIT. */
#define INFRABOT_CMD_MIT 0U                /* stream sample (any drive mode) */
#define INFRABOT_CMD_ENABLE 1U             /* both groups, IDLE */
#define INFRABOT_CMD_DISABLE 2U            /* both groups, immediately */
#define INFRABOT_CMD_SET_ZERO 3U
#define INFRABOT_CMD_SET_ZERO_ENABLE 4U
/* v3: controlled stop (brake, then hold in IDLE; resumable); v4: hold one motor. */
#define INFRABOT_CMD_STOP 5U
#define INFRABOT_CMD_CLEAR_ERROR 6U
#define INFRABOT_CMD_REQUEST_FEEDBACK 7U
#define INFRABOT_CMD_ENSURE_MODE 8U
#define INFRABOT_CMD_SEND_MIT 9U
#define INFRABOT_CMD_SEND_POS_VEL 10U
#define INFRABOT_CMD_SEND_VEL 11U
#define INFRABOT_CMD_SEND_FORCE_POS 12U
#define INFRABOT_CMD_SET_CAN_TIMEOUT 13U   /* register_value = ms */
#define INFRABOT_CMD_STORE_PARAMETERS 14U
#define INFRABOT_CMD_GET_REGISTER 15U
#define INFRABOT_CMD_WRITE_REGISTER 16U
#define INFRABOT_CMD_GET_STATE 17U
/* M1-M7 MIT or position-velocity (mode), M8 force-position; all disabled,
 * IDLE.  Ends with the parameter self-check (InfrabotSafetyStatus). */
#define INFRABOT_CMD_SET_ARM_MODE 18U
#define INFRABOT_CMD_STORE_CONTROL_PROFILE 19U /* DMCP v5 */
#define INFRABOT_CMD_SET_CONTROL_MODE 20U      /* mode = INFRABOT_CONTROL_* */
#define INFRABOT_CMD_HOLD_HEARTBEAT 21U        /* v3, 100 Hz in IDLE / NRT */
/* DMAR value = the motor's hold command, joint-frame mrad (int32). */
#define INFRABOT_CMD_GET_HOLD_POSITION 22U
#define INFRABOT_CMD_ENABLE_ARM 23U
#define INFRABOT_CMD_ENABLE_GRIPPER 24U
#define INFRABOT_CMD_DISABLE_ARM 25U
#define INFRABOT_CMD_DISABLE_GRIPPER 26U
/* v4, motor_index 7: target_position_mrad[7], velocity_limit_mrad_s (speed),
 * torque_ratio_milli (force = current limit per mille); any control mode. */
#define INFRABOT_CMD_GRIPPER 27U

/* Damiao drive modes (CTRL_MODE register, DMCP v4 mode, SET_ARM_MODE). */
#define INFRABOT_DRIVE_MODE_MIT 1U
#define INFRABOT_DRIVE_MODE_POS_VEL 2U
#define INFRABOT_DRIVE_MODE_VEL 3U
#define INFRABOT_DRIVE_MODE_FORCE_POS 4U

/* Control modes (SET_CONTROL_MODE, telemetry mode byte). */
#define INFRABOT_CONTROL_IDLE 0U
#define INFRABOT_CONTROL_RT_JOINT 1U
#define INFRABOT_CONTROL_RT_CARTESIAN 2U
#define INFRABOT_CONTROL_NRT_JOINT 3U
#define INFRABOT_CONTROL_NRT_CARTESIAN 4U
#define INFRABOT_CONTROL_GRAVITY_FREE 5U
#define INFRABOT_CONTROL_GRAVITY_LOCK 6U
#define INFRABOT_CONTROL_JOINT_IMPEDANCE 7U
#define INFRABOT_CONTROL_CARTESIAN_IMPEDANCE 8U
#define INFRABOT_CONTROL_JOINT_ADMITTANCE 9U
#define INFRABOT_CONTROL_CARTESIAN_ADMITTANCE 10U

/* Controller (operational) status, telemetry status byte bits 2-7. */
#define INFRABOT_STATUS_BOOTING 0U
#define INFRABOT_STATUS_PRECHECK 1U
#define INFRABOT_STATUS_PROFILE_REQUIRED 2U
#define INFRABOT_STATUS_DISABLED 3U
#define INFRABOT_STATUS_ENABLING 4U
#define INFRABOT_STATUS_IDLE_HOLD 5U
#define INFRABOT_STATUS_RT_JOINT 6U
#define INFRABOT_STATUS_RT_CARTESIAN 7U
#define INFRABOT_STATUS_NRT_JOINT 8U
#define INFRABOT_STATUS_NRT_CARTESIAN 9U
#define INFRABOT_STATUS_STOPPING 10U /* controlled stop: braking, or holding without a PC */
#define INFRABOT_STATUS_FAULT 11U
#define INFRABOT_STATUS_RECOVERY 12U
#define INFRABOT_STATUS_ESTOP 13U
#define INFRABOT_STATUS_UNKNOWN 14U
#define INFRABOT_STATUS_GRAVITY_FREE 15U
#define INFRABOT_STATUS_GRAVITY_LOCK 16U
#define INFRABOT_STATUS_JOINT_IMPEDANCE 17U
#define INFRABOT_STATUS_CARTESIAN_IMPEDANCE 18U
#define INFRABOT_STATUS_JOINT_ADMITTANCE 19U
#define INFRABOT_STATUS_CARTESIAN_ADMITTANCE 20U

/* DMAR status. */
#define INFRABOT_REPLY_OK 0U            /* done / verified */
#define INFRABOT_REPLY_QUEUED 1U        /* CAN frame queued, not a physical acknowledgement */
#define INFRABOT_REPLY_CAN_FAILED 2U    /* CAN failure / drive timeout */
#define INFRABOT_REPLY_INVALID 3U       /* argument out of range */
#define INFRABOT_REPLY_BUSY 4U          /* another operation in progress / not the owner */
#define INFRABOT_REPLY_DISABLE_FIRST 5U
/* Mode or readback mismatch.  SET_ARM_MODE: the parameter self-check failed,
 * value = joint | register << 8 | INFRABOT_SELFCHECK_* << 16. */
#define INFRABOT_REPLY_MISMATCH 6U
#define INFRABOT_REPLY_NOT_READY 7U     /* not enabled / feedback stale / drive fault */
#define INFRABOT_REPLY_PROTECTED 8U     /* read-only, protected register or value out of range */
#define INFRABOT_REPLY_UNSUPPORTED 9U
#define INFRABOT_SELFCHECK_REPLY_VALUE(joint, rid, result) \
  ((uint32_t)(joint) | ((uint32_t)(rid) << 8) | ((uint32_t)(result) << 16))

/* Why the MCU rejected a command datagram (InfrabotLinkStats.invalid_reason). */
#define INFRABOT_INVALID_FORMAT 1U        /* size/magic/version/CRC match no packet */
#define INFRABOT_INVALID_JOINT_STREAM 2U  /* joint stream outside enabled RT_JOINT */
#define INFRABOT_INVALID_STATE 3U         /* enable/zero not allowed in this state */
#define INFRABOT_INVALID_MOTION_OWNED 4U  /* a movej/movep owns the arm */
#define INFRABOT_INVALID_COMMAND 5U       /* unknown command code */
#define INFRABOT_INVALID_CARTESIAN 6U     /* Cartesian stream outside enabled RT_CARTESIAN */
#define INFRABOT_INVALID_LIMITS 7U        /* Cartesian speed limit above its maximum */
#define INFRABOT_INVALID_MAINTENANCE 8U   /* maintenance parameters invalid */
#define INFRABOT_INVALID_TIMED 9U         /* execute_mcu_us set where unsupported */

/* Damiao drive status (telemetry motor state: the status nibble of the
 * drive's feedback frame). */
#define INFRABOT_DRIVE_DISABLED 0x0U
#define INFRABOT_DRIVE_ENABLED 0x1U
#define INFRABOT_DRIVE_OVERVOLTAGE 0x8U
#define INFRABOT_DRIVE_UNDERVOLTAGE 0x9U
#define INFRABOT_DRIVE_OVERCURRENT 0xAU
#define INFRABOT_DRIVE_MOS_OVERTEMP 0xBU
#define INFRABOT_DRIVE_COIL_OVERTEMP 0xCU
#define INFRABOT_DRIVE_COMM_LOST 0xDU
#define INFRABOT_DRIVE_OVERLOAD 0xEU
#define INFRABOT_DRIVE_IS_FAULT(state) ((state) >= INFRABOT_DRIVE_OVERVOLTAGE)

/* DMIO capability bits. */
#define INFRABOT_CAP_JOINT_RT 0x00000001U
#define INFRABOT_CAP_JOINT_NRT 0x00000002U
#define INFRABOT_CAP_MAINTENANCE 0x00000004U
#define INFRABOT_CAP_PERSISTENCE 0x00000008U
#define INFRABOT_CAP_CARTESIAN_NRT 0x00000010U
#define INFRABOT_CAP_CARTESIAN_RT 0x00000020U
#define INFRABOT_CAP_GROUPS 0x00000040U          /* arm / gripper enabled separately */
#define INFRABOT_CAP_TELEMETRY_V3 0x00000080U
#define INFRABOT_CAP_CONTROLLED_STOP 0x00000100U /* InfrabotSafetyStatus, v3 STOP */
#define INFRABOT_CAP_SELFCHECK 0x00000200U       /* SET_ARM_MODE parameter self-check */
/* stop_mode reported; SET_CONTROL_MODE IDLE while in IDLE acknowledges a stop. */
#define INFRABOT_CAP_RESUME 0x00000400U
#define INFRABOT_CAP_LOCAL_GRAVITY 0x00000800U /* local 1 kHz gravity, no PC stream required */

#define INFRABOT_CAP_ADMITTANCE 0x00001000U /* local admittance, no target stream */
#define INFRABOT_CAP_RUN_LOG 0x00002000U    /* MCU run log on INFRABOT_LOG_PORT (DMLG) */
#define INFRABOT_CAP_PARAMETERS 0x00004000U /* parameter service (DMPR/DMPA) on the command port */
#define INFRABOT_CAP_DIAGNOSTICS 0x00008000U /* diagnostic snapshot (DMDR/DMDA) on the command port */

/* ---- Controlled stop -----------------------------------------------------------
 * On a stop condition the MCU leaves the control mode for IDLE, brakes the
 * joint command to rest (BRAKING), then holds the arm enabled.  It never
 * disables the drives by itself: with no PC it keeps holding (HOLDING) until
 * a PC is back -- a heartbeat or any accepted command makes it a normal IDLE
 * hold -- or sends DISABLE.  A joint that is blocked (NOT_FOLLOWING,
 * TORQUE_LIMIT) is held where it is instead of where it was commanded.
 * DISABLE removes power at once and ends a stop.
 *
 * stop_reason and stop_mode (the control mode the stop interrupted) stay
 * latched for the PC until the stop is acknowledged: by a successful
 * SET_CONTROL_MODE once the arm is stopped (phase NONE) -- the PC resumes by
 * entering stop_mode again, from where the arm stopped; a SET_CONTROL_MODE to
 * IDLE while in IDLE only acknowledges -- or by an enable.  A stop while an
 * earlier one is still latched happens in IDLE and keeps the earlier
 * stop_mode. */
#define INFRABOT_STOP_NONE 0U
#define INFRABOT_STOP_STREAM_LOST 1U    /* RT stream / IDLE heartbeat missing for 100 ms */
#define INFRABOT_STOP_MOVE_LOST 2U      /* movej/movep keepalive missing */
#define INFRABOT_STOP_MOVE_FAILED 3U    /* movej/movep failed, stop_detail = INFRABOT_MOVE_* reason */
#define INFRABOT_STOP_REQUESTED 4U      /* the PC asked: DMCP v3 STOP or a DMOV cancel */
#define INFRABOT_STOP_NOT_FOLLOWING 5U  /* RT Cartesian: a joint left behind its command */
#define INFRABOT_STOP_INVALID_TARGET 6U /* RT Cartesian: target rejected by the IK */
#define INFRABOT_STOP_FEEDBACK_LOST 7U  /* a drive stopped answering */
/* A drive reported a fault (stop_joint, stop_detail = INFRABOT_DRIVE_*), or
 * the CAN controller did (stop_joint INFRABOT_NO_JOINT). */
#define INFRABOT_STOP_DRIVE_FAULT 8U
#define INFRABOT_STOP_TORQUE_LIMIT 9U   /* a joint at its torque limit for too long */

#define INFRABOT_STOP_PHASE_NONE 0U     /* no stop in progress (normal operation) */
#define INFRABOT_STOP_PHASE_BRAKING 1U
#define INFRABOT_STOP_PHASE_HOLDING 2U  /* stopped, holding, no PC yet */

/* Parameter self-check of SET_ARM_MODE: every drive's MST_ID, ESC_ID,
 * CTRL_MODE, PMAX, VMAX and TMAX read back and compared with the firmware's
 * table (the MIT encoding and telemetry scaling use it).  A failure keeps the
 * profile not ready, so the arm cannot be enabled.  J8 is checked when
 * connected. */
#define INFRABOT_SELFCHECK_NOT_RUN 0U
#define INFRABOT_SELFCHECK_OK 1U
#define INFRABOT_SELFCHECK_MISMATCH 2U
#define INFRABOT_SELFCHECK_NO_REPLY 3U

/* Telemetry v3 per-motor flags. */
#define INFRABOT_MOTOR_ENABLED 0x01U      /* the MCU drives it */
#define INFRABOT_MOTOR_OVERRIDE 0x02U     /* a maintenance command / move owns it */
#define INFRABOT_MOTOR_STOPPED 0x04U      /* held by a v4 STOP */
#define INFRABOT_MOTOR_TORQUE_LIMIT 0x08U /* measured torque at the supervisor limit */
#define INFRABOT_MOTOR_FEEDBACK_STALE 0x10U

/* Telemetry status byte. */
#define INFRABOT_STATUS_FLAG_ENABLED 0x01U    /* any drive enabled */
#define INFRABOT_STATUS_FLAG_ZERO_VALID 0x02U
#define INFRABOT_STATUS_FROM_BYTE(status) (((status) >> 2) & 0x3FU)
/* Telemetry mode byte. */
#define INFRABOT_MODE_FROM_BYTE(mode) ((mode) & 0x0FU)
#define INFRABOT_PROFILE_FROM_BYTE(mode) (((mode) >> 4) & 0x03U)
#define INFRABOT_MODE_FLAG_PROFILE_READY 0x40U
#define INFRABOT_MODE_FLAG_SUPPORTED 0x80U /* firmware with control modes */

/* ---- DMOV movej / movep --------------------------------------------------------- */
#define INFRABOT_MOVE_OP_MOVEJ 1U
#define INFRABOT_MOVE_OP_MOVEP 2U
#define INFRABOT_MOVE_OP_KEEPALIVE 3U /* 100 Hz while a move owns the arm */
#define INFRABOT_MOVE_OP_CANCEL 4U    /* controlled stop, IDLE hold (resumable) */
#define INFRABOT_MOVE_OP_QUERY 5U

#define INFRABOT_MOVE_STATE_IDLE 0U
#define INFRABOT_MOVE_STATE_RUNNING 1U
#define INFRABOT_MOVE_STATE_SETTLING 2U
#define INFRABOT_MOVE_STATE_DONE 3U
#define INFRABOT_MOVE_STATE_FAILED 4U
#define INFRABOT_MOVE_STATE_CANCELLED 5U

/* DMOS result / reason. */
#define INFRABOT_MOVE_OK 0U
#define INFRABOT_MOVE_BUSY 1U
#define INFRABOT_MOVE_NOT_READY 2U
#define INFRABOT_MOVE_INVALID 3U
#define INFRABOT_MOVE_UNSUPPORTED 4U
#define INFRABOT_MOVE_BAD_SESSION 5U
#define INFRABOT_MOVE_STALE_ID 6U
#define INFRABOT_MOVE_COMM_TIMEOUT 7U
#define INFRABOT_MOVE_MOTION_TIMEOUT 8U
#define INFRABOT_MOVE_FEEDBACK_STALE 9U
#define INFRABOT_MOVE_MOTOR_FAULT 10U
#define INFRABOT_MOVE_CANCEL 11U
#define INFRABOT_MOVE_TX_ERROR 12U

/* Cartesian speed limits on the wire (v7, movep): {mm/s, mrad/s, mm/s^2,
 * mrad/s^2}, 0 = MCU default; maxima 0.5 m/s, 180 deg/s, 2 m/s^2, 360 deg/s^2. */
#define INFRABOT_CARTESIAN_MAX_LINEAR_VELOCITY 500U
#define INFRABOT_CARTESIAN_MAX_ANGULAR_VELOCITY 3142U
#define INFRABOT_CARTESIAN_MAX_LINEAR_ACCELERATION 2000U
#define INFRABOT_CARTESIAN_MAX_ANGULAR_ACCELERATION 6283U

/* ---- Datagrams ---------------------------------------------------------------- */

/* DMCP v2 (legacy): positions only. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_COMMAND_V2 */
  uint16_t motor_count;
  uint32_t sequence;
  uint64_t pc_timestamp_us;
  uint64_t execute_mcu_us;
  int32_t target_position_mrad[INFRABOT_MOTOR_COUNT];
  uint32_t crc32;
} InfrabotCommandV2;

/* DMCP v3: a control command (ENABLE, DISABLE, STOP, groups, heartbeat) or,
 * with INFRABOT_CMD_MIT, a position sample (execute_mcu_us 0 = now). */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_COMMAND_V3 */
  uint16_t motor_count;
  uint32_t sequence;
  uint8_t command;  /* INFRABOT_CMD_* */
  uint8_t reserved[3];
  uint64_t pc_timestamp_us;
  uint64_t execute_mcu_us;
  int32_t target_position_mrad[INFRABOT_MOTOR_COUNT];
  uint32_t crc32;
} InfrabotCommand;

/* DMCP v4: one low-rate maintenance operation, answered by a DMAR with the
 * same sequence.  The MCU runs one at a time (DISABLE excepted). */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_COMMAND_V4 */
  uint16_t motor_count;
  uint32_t sequence;
  uint8_t command;     /* INFRABOT_CMD_* */
  uint8_t motor_index; /* 0-7 or INFRABOT_MOTOR_ALL */
  uint8_t mode;        /* INFRABOT_DRIVE_MODE_* / INFRABOT_CONTROL_* */
  uint8_t value_type;  /* INFRABOT_REGISTER_U32 / _F32 */
  uint8_t register_id;
  uint8_t reserved[3];
  uint64_t pc_timestamp_us;
  uint64_t execute_mcu_us; /* 0 */
  int32_t target_position_mrad[INFRABOT_MOTOR_COUNT]; /* [motor_index] */
  int32_t target_velocity_mrad_s;
  int32_t kp_milli;
  int32_t kd_milli;
  int32_t torque_mNm;
  int32_t velocity_limit_mrad_s;
  int32_t torque_ratio_milli;
  uint32_t register_value;
  uint32_t crc32;
} InfrabotMaintenance;

/* DMCP v5: the RT joint stream (command MIT, 1 kHz) or the control-profile
 * store (command STORE_CONTROL_PROFILE).  J1-J7 per joint: MIT drives track
 * position/velocity with kp/kd plus the torque feed-forward; position-velocity
 * drives move to the position at no more than velocity_limit (their v_des).
 * The gripper fields are ignored (INFRABOT_CMD_GRIPPER). */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_COMMAND_V5 */
  uint16_t motor_count;
  uint32_t sequence;
  uint8_t command;
  uint8_t reserved[3];
  uint64_t pc_timestamp_us;
  uint64_t execute_mcu_us; /* 0 */
  int32_t target_position_mrad[INFRABOT_MOTOR_COUNT];
  int32_t target_velocity_mrad_s[INFRABOT_MOTOR_COUNT];
  int32_t kp_milli[INFRABOT_MOTOR_COUNT];
  int32_t kd_milli[INFRABOT_MOTOR_COUNT];
  int32_t torque_mNm[INFRABOT_MOTOR_COUNT];
  int32_t velocity_limit_mrad_s[INFRABOT_MOTOR_COUNT];
  int32_t torque_ratio_milli[INFRABOT_MOTOR_COUNT];
  uint32_t crc32;
} InfrabotJointStream;

/* DMCP v7: the RT Cartesian stream.  pose = xyz metres + row-major 3x3
 * rotation of the tool in the base frame; the MCU moves the tool to it along
 * a straight line within cartesian_limits. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_COMMAND_V7 */
  uint16_t motor_count;
  uint32_t sequence;
  uint8_t command;  /* INFRABOT_CMD_MIT */
  uint8_t reserved[3];
  uint64_t pc_timestamp_us;
  uint64_t execute_mcu_us; /* 0 */
  float pose[12];
  int32_t target_velocity_mrad_s[INFRABOT_MOTOR_COUNT];
  int32_t kp_milli[INFRABOT_MOTOR_COUNT];
  int32_t kd_milli[INFRABOT_MOTOR_COUNT];
  int32_t torque_mNm[INFRABOT_MOTOR_COUNT];
  int32_t velocity_limit_mrad_s[INFRABOT_MOTOR_COUNT];
  int32_t torque_ratio_milli[INFRABOT_MOTOR_COUNT];
  uint16_t cartesian_limits[4];
  uint32_t crc32;
} InfrabotCartesianStream;

/* DMAR: the result of a DMCP v4 operation (or v5 profile store). */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_MAINTENANCE_V1 */
  uint8_t command;
  uint8_t motor_index;
  uint8_t status;   /* INFRABOT_REPLY_* */
  uint8_t value_type;
  uint8_t register_id;
  uint8_t reserved;
  uint32_t sequence; /* of the request */
  uint32_t value;
  uint32_t crc32;
} InfrabotMaintenanceReply;

/* DMIF / DMIO: controller information, before enabling. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_INFO_V1 or INFRABOT_INFO_V2 */
  uint16_t reserved;
  uint32_t sequence;
  uint32_t crc32;
} InfrabotInfoRequest;

typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version;
  uint8_t status;
  uint8_t controller_status;
  uint8_t control_mode;
  uint8_t motor_profile;
  uint8_t profile_ready;
  uint8_t enabled_mask;
  uint8_t axes;
  uint8_t motor_count;
  uint32_t sequence;
  uint32_t capabilities; /* INFRABOT_CAP_* */
  float position_min[INFRABOT_MOTOR_COUNT];
  float position_max[INFRABOT_MOTOR_COUNT];
  float velocity_max[INFRABOT_MOTOR_COUNT];
  float torque_max[INFRABOT_MOTOR_COUNT];
  uint32_t crc32;
} InfrabotInfoReply;

/* V2 retains the complete info prefix, with version=2 and its own prefix
 * CRC. The final CRC covers the prefix, its CRC, and every extension byte.
 * The producer NUL-terminates strings; readers must bound all reads. */
typedef struct INFRABOT_PACKED {
  InfrabotInfoReply info;
  uint16_t protocol_major;
  uint16_t protocol_minor;
  char firmware_version[24];
  char board[16];
  char arm_model[16];
  uint32_t crc32;
} InfrabotInfoReplyV2;


/* DMCS / DMCR: PC <-> MCU clock offset, to the telemetry port. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t sequence;
  uint64_t pc_timestamp_us;
  uint32_t crc32;
} InfrabotSyncRequest;

typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t sequence;
  uint64_t pc_timestamp_us;
  uint64_t mcu_rx_timestamp_us;
  uint64_t mcu_tx_timestamp_us;
  uint32_t crc32;
} InfrabotSyncReply;

/* DMTL: measured state of one drive (joint frame). */
typedef struct INFRABOT_PACKED {
  int32_t position_mrad;
  int32_t velocity_mrad_s;
  int32_t torque_mNm;
  uint8_t state;           /* INFRABOT_DRIVE_* */
  uint8_t temperature_mos; /* degC */
  uint16_t feedback_count; /* low 16 bits of the drive's feedback frame count */
} InfrabotTelemetryMotor;

/* DMTL v2 (180 bytes), also the head of v3. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;
  uint16_t version; /* INFRABOT_TELEMETRY_V2 / _V3 */
  uint16_t motor_count;
  uint32_t sequence;
  uint32_t timestamp_ms;
  uint64_t mcu_timestamp_us;
  InfrabotTelemetryMotor motor[INFRABOT_MOTOR_COUNT];
  uint32_t command_last_sequence;
  uint32_t command_applied_sequence;
  uint8_t command_active;
  uint8_t status;       /* INFRABOT_STATUS_FLAG_*, status << 2 */
  uint8_t enabled_mask; /* bit i: J(i+1) enabled */
  uint8_t mode;         /* control mode, profile << 4, INFRABOT_MODE_FLAG_* */
  uint64_t command_last_rx_mcu_us;
  uint64_t command_applied_mcu_us;
} InfrabotTelemetry;

/* DMTL v3: what the MCU commands each drive this tick (joint frame). */
typedef struct INFRABOT_PACKED {
  int32_t position_mrad;   /* the planned command (position-velocity: before the lag compensation) */
  int32_t velocity_mrad_s; /* MIT velocity feed-forward; position-velocity/force-position: v_des */
  int32_t torque_mNm;      /* MIT torque feed-forward, 0 in the other modes */
  uint8_t temperature_coil; /* degC */
  uint8_t flags;           /* INFRABOT_MOTOR_* */
  uint8_t drive_mode;      /* INFRABOT_DRIVE_MODE_* */
  uint8_t reserved;
} InfrabotTelemetryCommand;

/* DMTL v3: the MCU's view of the command link, over the previous second
 * (refreshed once a second; values saturate at 65535). */
typedef struct INFRABOT_PACKED {
  uint16_t command_rx;              /* command datagrams accepted */
  uint16_t command_invalid;         /* command datagrams rejected */
  uint16_t command_interval_max_us; /* longest gap between command datagrams */
  uint16_t command_jitter_max_us;   /* largest deviation from 1 ms */
  uint16_t control_missed;          /* 1 kHz control ticks missed */
  uint16_t control_period_max_us;
  uint16_t can_tx_errors;           /* CAN frames that could not be queued */
  uint16_t telemetry_errors;        /* telemetry/reply datagrams not sent */
  uint8_t invalid_reason;           /* INFRABOT_INVALID_* of the last rejected datagram */
  uint8_t reserved[3];
} InfrabotLinkStats;

/* DMTL v3: controlled stop and parameter self-check. */
typedef struct INFRABOT_PACKED {
  uint8_t stop_reason;        /* INFRABOT_STOP_*, latched */
  uint8_t stop_phase;         /* INFRABOT_STOP_PHASE_* */
  uint8_t stop_joint;         /* joint involved (0-based), INFRABOT_NO_JOINT */
  uint8_t stop_detail;        /* INFRABOT_MOVE_* / INFRABOT_DRIVE_* / IK status (see the reason) */
  uint8_t selfcheck;          /* INFRABOT_SELFCHECK_* of the last SET_ARM_MODE */
  uint8_t selfcheck_joint;    /* first joint that failed (0-based), INFRABOT_NO_JOINT */
  uint8_t selfcheck_register; /* its register (RID) */
  /* The control mode the latched stop interrupted (INFRABOT_CONTROL_*),
   * which a resume re-enters; IDLE when it happened in IDLE (or none). */
  uint8_t stop_mode;
  uint8_t reserved[4];
  uint32_t selfcheck_value;   /* raw register value read (MISMATCH) */
  uint32_t selfcheck_expected; /* raw value of the firmware's table */
} InfrabotSafetyStatus;

typedef struct INFRABOT_PACKED {
  InfrabotTelemetry head; /* version INFRABOT_TELEMETRY_V3 */
  InfrabotTelemetryCommand command[INFRABOT_MOTOR_COUNT];
  InfrabotLinkStats link;
  InfrabotSafetyStatus safety;
} InfrabotTelemetryV3;

/* DMOV payloads (naturally aligned, no padding: identical in memory and on
 * the wire).  A joint move goes stop-to-stop to target within the per-joint
 * velocity/acceleration caps; a Cartesian move along a straight line to pose
 * (xyz metres + row-major rotation) within cartesian_limits, the per-joint
 * caps only bounding it. */
typedef struct {
  uint32_t timeout_ms, dwell_ms;
  float position_tolerance, velocity_tolerance;
  float target[INFRABOT_ARM_AXES], velocity_max[INFRABOT_ARM_AXES];
  float acceleration_max[INFRABOT_ARM_AXES], kp[INFRABOT_ARM_AXES];
  float kd[INFRABOT_ARM_AXES], torque[INFRABOT_ARM_AXES];
} InfrabotMoveJoint;

typedef struct {
  uint32_t timeout_ms, dwell_ms;
  float position_tolerance, velocity_tolerance;
  float pose[12];
  float velocity_max[INFRABOT_ARM_AXES], acceleration_max[INFRABOT_ARM_AXES];
  float kp[INFRABOT_ARM_AXES], kd[INFRABOT_ARM_AXES];
  uint16_t cartesian_limits[4];
} InfrabotMoveCartesian;

typedef struct {
  uint32_t magic;   /* INFRABOT_MAGIC_MOVE_REQUEST */
  uint16_t version; /* INFRABOT_MOVE_V1 */
  uint8_t op;       /* INFRABOT_MOVE_OP_* */
  uint8_t reserved;
  uint64_t session;
  uint32_t motion_id;
  union {
    InfrabotMoveJoint joint;         /* MOVEJ */
    InfrabotMoveCartesian cartesian; /* MOVEP */
  } command;                         /* zero for the other ops */
  uint32_t crc32;
} InfrabotMoveRequest;

typedef struct {
  uint32_t magic;   /* INFRABOT_MAGIC_MOVE_REPLY */
  uint16_t version;
  uint8_t result;   /* INFRABOT_MOVE_* of this request */
  uint8_t state;    /* INFRABOT_MOVE_STATE_* */
  uint64_t session; /* echoed */
  uint32_t request_id, active_id, elapsed_ms, duration_ms;
  uint32_t arrived_mask, enabled_mask, fresh_mask;
  float max_error_rad;
  uint32_t reason;  /* INFRABOT_MOVE_* of the session */
  uint32_t crc32;
} InfrabotMoveReply;

/* ---- MCU run log (DMLG -> DMLR on INFRABOT_LOG_PORT) --------------------------
 * The MCU keeps two rings in SDRAM, lost on power-off and on reset:
 *   text      every line printf sends to USART1, newest 10 minutes;
 *   telemetry one DMTL v3 snapshot per control tick, newest 10 s (sequence =
 *             record index; mcu_timestamp_us on the DMTL clock).
 * Pull only: one DMLR per DMLG, at most a few per millisecond, refused while a
 * PC real-time stream (RT_JOINT / RT_CARTESIAN) is active.
 * TEXT data: whole records [u32 seq][u32 time_ms][u16 length][text][pad to 4].
 * TELEMETRY data: whole InfrabotTelemetryV3 records.
 * FREEZE pauses telemetry recording so a download is consistent; RESUME or
 * INFRABOT_LOG_FREEZE_TIMEOUT_MS without requests restarts it. */
#define INFRABOT_LOG_V1 1U
#define INFRABOT_LOG_OP_STATUS 0U
#define INFRABOT_LOG_OP_TEXT 1U       /* cursor = first text seq wanted */
#define INFRABOT_LOG_OP_TELEMETRY 2U  /* cursor = first record index wanted */
#define INFRABOT_LOG_OP_FREEZE 3U
#define INFRABOT_LOG_OP_RESUME 4U
#define INFRABOT_LOG_OK 0U
#define INFRABOT_LOG_UNAVAILABLE 1U   /* SDRAM not running: no log */
#define INFRABOT_LOG_BUSY 2U          /* PC real-time stream active */
#define INFRABOT_LOG_INVALID 3U
#define INFRABOT_LOG_TEXT_HEADER 10U
#define INFRABOT_LOG_DATA_MAX 1392U   /* 4 DMTL v3; 64 + 1392 fits one 1472-byte UDP payload */

typedef struct INFRABOT_PACKED {
  uint32_t magic;       /* INFRABOT_MAGIC_LOG_REQUEST */
  uint16_t version;     /* INFRABOT_LOG_V1 */
  uint8_t op;           /* INFRABOT_LOG_OP_* */
  uint8_t reserved;
  uint32_t request_id;  /* echoed */
  uint32_t cursor;
  uint32_t max_bytes;   /* 0 or > INFRABOT_LOG_DATA_MAX: INFRABOT_LOG_DATA_MAX */
} InfrabotLogRequest;

typedef struct INFRABOT_PACKED {
  uint32_t magic;       /* INFRABOT_MAGIC_LOG_REPLY */
  uint16_t version;
  uint8_t op;
  uint8_t result;       /* INFRABOT_LOG_* */
  uint32_t request_id;
  uint32_t boot_id;     /* differs after an MCU reset: the rings restarted */
  uint32_t now_ms;      /* text time_ms clock (HAL tick) */
  uint32_t writer_max_us; /* STATUS: longest log-task tick since the previous STATUS */
  uint64_t now_us;      /* DMTL mcu_timestamp_us clock (follows writer_max_us) */
  uint32_t text_first, text_next;            /* kept text seq [first, next) */
  uint32_t telemetry_first, telemetry_next;  /* kept record index [first, next) */
  uint32_t cursor;      /* first seq/index in data (after clamping to first) */
  uint32_t next_cursor; /* request this next */
  uint16_t data_bytes;
  uint8_t record_count;
  uint8_t frozen;       /* bit0: paused for a download (expires); bit1: latched by a fault until a RESUME */
  uint32_t lost_bytes;  /* printf bytes the log could not keep (since boot) */
} InfrabotLogReplyHeader;

INFRABOT_STATIC_ASSERT(sizeof(InfrabotLogRequest) == 20U, "DMLG size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotLogReplyHeader) == 64U, "DMLR header size");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotLogReplyHeader, now_us) == 24U, "DMLR now_us offset");

/* ---- Parameter service (DMPR -> DMPA on INFRABOT_COMMAND_PORT) ---------------
 * Reads and writes the controller's tunable parameters (gravity, impedance and
 * admittance settings) by id, the same values the touchscreen sliders change.
 * One request in, one reply out, sent from the command port to the sender.
 * Every parameter describes itself, so the PC lists them with INFO instead of
 * compiling in a table, and a firmware that adds parameters needs no new
 * protocol version.
 *
 * param_id = group << 8 | index.  INFO takes a list position in param_id (0 ..
 * param_count - 1) and returns that parameter's descriptor; GET and SET take
 * the id.  SET stores a requested value; the control task slews the applied
 * value toward it at the parameter's rate, so a write never steps a gain.
 * Values are RAM only: they return to their defaults at reboot.  SAVE is
 * reserved for persisting them.  Only the controlling PC may SET or SAVE
 * (INFRABOT_PARAM_BUSY otherwise); INFO and GET are open to any PC. */
#define INFRABOT_PARAM_OP_INFO 0U
#define INFRABOT_PARAM_OP_GET 1U
#define INFRABOT_PARAM_OP_SET 2U
#define INFRABOT_PARAM_OP_SAVE 3U

#define INFRABOT_PARAM_GROUP_GRAVITY 1U
#define INFRABOT_PARAM_GROUP_IMPEDANCE 2U
#define INFRABOT_PARAM_GROUP_ADMITTANCE 3U
#define INFRABOT_PARAM_ID(group, index) ((uint16_t)(((group) << 8) | (index)))
#define INFRABOT_PARAM_ID_GROUP(id) ((uint8_t)((id) >> 8))
#define INFRABOT_PARAM_ID_INDEX(id) ((uint8_t)((id) & 0xFFU))

#define INFRABOT_PARAM_OK 0U
#define INFRABOT_PARAM_UNKNOWN 1U     /* no such parameter or list position */
#define INFRABOT_PARAM_RANGE 2U       /* SET value not finite or outside [minimum, maximum] */
#define INFRABOT_PARAM_READ_ONLY 3U
#define INFRABOT_PARAM_BUSY 4U        /* another PC controls the arm */
#define INFRABOT_PARAM_UNSUPPORTED 5U /* the operation is not available in this firmware */
#define INFRABOT_PARAM_INVALID 6U     /* unknown op or reserved field not zero */

#define INFRABOT_PARAM_FLAG_WRITABLE 0x01U
#define INFRABOT_PARAM_FLAG_BOOLEAN 0x02U /* value 0 or 1 */
#define INFRABOT_PARAM_NAME_LENGTH 24U
#define INFRABOT_PARAM_UNIT_LENGTH 8U

typedef struct INFRABOT_PACKED {
  uint32_t magic;       /* INFRABOT_MAGIC_PARAM_REQUEST */
  uint16_t version;     /* INFRABOT_PARAM_V1 */
  uint8_t op;           /* INFRABOT_PARAM_OP_* */
  uint8_t reserved0;    /* 0 */
  uint32_t request_id;  /* echoed */
  uint16_t param_id;    /* INFO: list position; GET, SET: id */
  uint16_t reserved1;   /* 0 */
  float value;          /* SET: the requested value; otherwise 0 */
  uint32_t crc32;
} InfrabotParamRequest;

/* Every reply carries the descriptor of the addressed parameter.  On an error
 * the descriptor fields are zero except param_count and request_id. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;       /* INFRABOT_MAGIC_PARAM_REPLY */
  uint16_t version;
  uint8_t op;           /* of the request */
  uint8_t result;       /* INFRABOT_PARAM_* */
  uint32_t request_id;
  uint16_t param_id;
  uint16_t param_count; /* parameters in the firmware */
  float requested;      /* last SET value, or the default after reboot */
  float applied;        /* value the controller uses now (slewing toward requested) */
  float minimum;
  float maximum;
  float default_value;
  float rate;           /* largest change of the applied value, units per second */
  uint8_t flags;        /* INFRABOT_PARAM_FLAG_* */
  uint8_t group;        /* INFRABOT_PARAM_GROUP_* */
  uint16_t reserved;    /* 0 */
  char name[INFRABOT_PARAM_NAME_LENGTH];  /* NUL-terminated */
  char unit[INFRABOT_PARAM_UNIT_LENGTH];  /* NUL-terminated, e.g. "Nm/rad" */
  uint32_t crc32;
} InfrabotParamReply;

INFRABOT_STATIC_ASSERT(sizeof(InfrabotParamRequest) == 24U, "DMPR size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotParamReply) == 80U, "DMPA size");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotParamReply, name) == 44U, "DMPA name offset");

/* ---- Diagnostic snapshot (DMDR -> DMDA on INFRABOT_COMMAND_PORT) ------------
 * Read-only health of the controller, open to any PC and never taking control
 * of the arm.  One request in, one reply out, sent from the command port to the
 * sender.  The per-second fields are the status task's last one-second window
 * (snapshot_seconds tells which; 0 before the first); the totals count since
 * boot.  Stack figures are the least free stack each task has ever had. */
typedef struct INFRABOT_PACKED {
  uint32_t magic;      /* INFRABOT_MAGIC_DIAG_REQUEST */
  uint16_t version;    /* INFRABOT_DIAG_V1 */
  uint16_t reserved;   /* 0 */
  uint32_t request_id; /* echoed */
  uint32_t crc32;
} InfrabotDiagRequest;

typedef struct INFRABOT_PACKED {
  uint32_t magic;      /* INFRABOT_MAGIC_DIAG_REPLY */
  uint16_t version;
  uint16_t reserved0;
  uint32_t request_id;
  uint32_t uptime_ms;             /* since the application started; wraps after 49 days */
  uint32_t snapshot_seconds;      /* status task second of the window below */
  /* The last one-second window. */
  uint32_t control_hz;            /* 1 kHz control ticks completed */
  uint32_t control_missed;
  uint32_t control_period_max_us;
  uint32_t can_tx;                /* drive frames queued */
  uint32_t can_rx;                /* frames received */
  uint32_t can_tx_errors;         /* drive frames that could not be queued */
  uint32_t command_rx;            /* command datagrams accepted */
  uint32_t command_invalid;
  uint32_t telemetry_tx;
  /* Since boot. */
  uint32_t can_error_events;
  uint32_t control_missed_total;
  uint32_t can_tx_errors_total;
  uint32_t command_invalid_total;
  /* CAN controller state at the end of the window. */
  uint8_t can_tec;
  uint8_t can_rec;
  uint8_t can_error_passive;
  uint8_t can_bus_off;
  /* Memory. */
  uint32_t heap_free_bytes;
  uint32_t heap_min_free_bytes;   /* lowest since boot */
  uint16_t stack_free_control_bytes;
  uint16_t stack_free_status_bytes;
  uint16_t stack_free_ethernet_bytes;
  uint16_t reserved1;
  uint8_t drive_state[8];         /* INFRABOT_DRIVE_* of J1..J8, now */
  uint8_t controller_status;      /* INFRABOT_STATUS_*, now */
  uint8_t invalid_reason;         /* INFRABOT_INVALID_* of the last rejected command datagram */
  uint16_t reserved2;
  uint32_t crc32;
} InfrabotDiagReply;

INFRABOT_STATIC_ASSERT(sizeof(InfrabotDiagRequest) == 16U, "DMDR size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotDiagReply) == 108U, "DMDA size");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotDiagReply, drive_state) == 92U, "DMDA drive_state offset");

INFRABOT_STATIC_ASSERT(sizeof(InfrabotCommandV2) == 64U, "DMCP v2 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotCommand) == 68U, "DMCP v3 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMaintenance) == 100U, "DMCP v4 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotJointStream) == 260U, "DMCP v5 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotCartesianStream) == 284U, "DMCP v7 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMaintenanceReply) == 24U, "DMAR size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotInfoRequest) == 16U, "DMIF size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotInfoReply) == 154U, "DMIO size");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotInfoReplyV2, protocol_major) == 154U, "DMIO extension");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotInfoReplyV2) == 218U, "DMIO v2 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotSyncRequest) == 24U, "DMCS size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotSyncReply) == 40U, "DMCR size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotTelemetryMotor) == 16U, "DMTL motor size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotTelemetry) == 180U, "DMTL v2 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotTelemetryCommand) == 16U, "DMTL v3 command size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotLinkStats) == 20U, "DMTL v3 link size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotSafetyStatus) == 20U, "DMTL v3 safety size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotTelemetryV3) == 348U, "DMTL v3 size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMoveJoint) == 184U, "DMOV joint payload size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMoveCartesian) == 184U, "DMOV Cartesian payload size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMoveRequest) == 208U, "DMOV request size");
INFRABOT_STATIC_ASSERT(sizeof(InfrabotMoveReply) == 56U, "DMOS reply size");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotMoveRequest, command) == 20U, "DMOV payload offset");
INFRABOT_STATIC_ASSERT(offsetof(InfrabotMoveReply, reason) == 48U, "DMOS reason offset");

static inline uint32_t infrabot_crc32(const void *data, size_t length)
{
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0U; i < length; ++i)
  {
    crc ^= bytes[i];
    for (unsigned bit = 0U; bit < 8U; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

/* ---- Damiao drive registers ---------------------------------------------------
 * X(rid, name, type, access, low, high, bounds, description): the one table
 * of the registers the MCU reads and writes (DMCP v4 GET/WRITE_REGISTER), the
 * GUI and the motor template importer check against.  type is the DMCP v4
 * value_type.  access: READ_ONLY; WRITABLE through WRITE_REGISTER while every
 * drive is disabled; PROTECTED: writable on the drive but refused by the MCU
 * (changing it live breaks the CAN addressing or the feedback decoder).  A
 * write must lie in [low, high], or the open side of bounds. */
#define INFRABOT_REGISTER_U32 0U
#define INFRABOT_REGISTER_F32 1U
#define INFRABOT_REGISTER_READ_ONLY 0U
#define INFRABOT_REGISTER_WRITABLE 1U
#define INFRABOT_REGISTER_PROTECTED 2U
#define INFRABOT_REGISTER_CLOSED 0U
#define INFRABOT_REGISTER_LOW_OPEN 1U
#define INFRABOT_REGISTER_HIGH_OPEN 2U
#define INFRABOT_REGISTER_OPEN 3U
#define INFRABOT_FLOAT_MAX 3.4e38f

#define INFRABOT_RID_MST_ID 7U
#define INFRABOT_RID_ESC_ID 8U
#define INFRABOT_RID_TIMEOUT 9U /* CAN timeout, units of 50 us (ms x 20); 0 = off */
#define INFRABOT_RID_CTRL_MODE 10U
#define INFRABOT_RID_PMAX 21U
#define INFRABOT_RID_VMAX 22U
#define INFRABOT_RID_TMAX 23U
#define INFRABOT_RID_KP_APR 27U
#define INFRABOT_RID_CAN_BR 35U

#define INFRABOT_REGISTERS(X) \
  X(0U, "UV_Value", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 10.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "低压保护值 V") \
  X(1U, "KT_Value", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_CLOSED, "力矩系数") \
  X(2U, "OT_Value", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 80.0f, 200.0f, INFRABOT_REGISTER_HIGH_OPEN, "过温保护值 °C") \
  X(3U, "OC_Value", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, 1.0f, INFRABOT_REGISTER_OPEN, "过流保护值（相对最大电流）") \
  X(4U, "ACC", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "加速度") \
  X(5U, "DEC", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, -INFRABOT_FLOAT_MAX, 0.0f, INFRABOT_REGISTER_HIGH_OPEN, "减速度") \
  X(6U, "MAX_SPD", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "最大速度") \
  X(7U, "MST_ID", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_PROTECTED, 0.0f, 2047.0f, INFRABOT_REGISTER_CLOSED, "反馈 ID") \
  X(8U, "ESC_ID", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_PROTECTED, 0.0f, 2047.0f, INFRABOT_REGISTER_CLOSED, "CAN 接收 ID") \
  X(9U, "TIMEOUT", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_WRITABLE, 20.0f, 4294967295.0f, INFRABOT_REGISTER_CLOSED, "CAN 超时（×50 µs）") \
  X(10U, "CTRL_MODE", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_WRITABLE, 1.0f, 4.0f, INFRABOT_REGISTER_CLOSED, "控制模式 1 MIT/2 位置速度/3 速度/4 力位") \
  X(11U, "Damp", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "粘滞阻尼系数") \
  X(12U, "Inertia", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "转动惯量") \
  X(13U, "hw_ver", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "硬件版本") \
  X(14U, "sw_ver", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "软件版本") \
  X(15U, "SN", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "序列号") \
  X(16U, "NPP", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "极对数") \
  X(17U, "Rs", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "相电阻") \
  X(18U, "Ls", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "相电感") \
  X(19U, "Flux", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "磁链") \
  X(20U, "Gr", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "减速比") \
  X(21U, "PMAX", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "位置映射范围 rad") \
  X(22U, "VMAX", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "速度映射范围 rad/s") \
  X(23U, "TMAX", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "扭矩映射范围 N·m") \
  X(24U, "I_BW", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 100.0f, 10000.0f, INFRABOT_REGISTER_CLOSED, "电流环带宽") \
  X(25U, "KP_ASR", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_CLOSED, "速度环 Kp") \
  X(26U, "KI_ASR", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_CLOSED, "速度环 Ki") \
  X(27U, "KP_APR", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_CLOSED, "位置环 Kp") \
  X(28U, "KI_APR", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_CLOSED, "位置环 Ki") \
  X(29U, "OV_Value", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, INFRABOT_FLOAT_MAX, INFRABOT_REGISTER_LOW_OPEN, "过压保护值 V") \
  X(30U, "GREF", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, 1.0f, INFRABOT_REGISTER_LOW_OPEN, "齿轮力矩效率") \
  X(31U, "Deta", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 1.0f, 30.0f, INFRABOT_REGISTER_CLOSED, "速度环阻尼系数") \
  X(32U, "V_BW", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, 500.0f, INFRABOT_REGISTER_OPEN, "速度环滤波带宽") \
  X(33U, "IQ_c1", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 100.0f, 10000.0f, INFRABOT_REGISTER_CLOSED, "电流环增强系数") \
  X(34U, "VL_c1", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_WRITABLE, 0.0f, 10000.0f, INFRABOT_REGISTER_LOW_OPEN, "速度环增强系数") \
  X(35U, "can_br", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_PROTECTED, 0.0f, 11.0f, INFRABOT_REGISTER_CLOSED, "CAN 波特率代码") \
  X(36U, "sub_ver", INFRABOT_REGISTER_U32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "子版本号") \
  X(50U, "u_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "U 相偏置") \
  X(51U, "v_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "V 相偏置") \
  X(52U, "k1", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "补偿系数 1") \
  X(53U, "k2", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "补偿系数 2") \
  X(54U, "m_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "角度偏置") \
  X(55U, "dir", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "方向") \
  X(56U, "m_off2", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "电机侧角度偏置") \
  X(59U, "Imax", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "驱动板最大电流 A") \
  X(60U, "VBus", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "母线电压 V") \
  X(61U, "Tpcb", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "驱动板温度 °C") \
  X(62U, "Tmtr", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "电机温度 °C") \
  X(63U, "Iu_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "U 相电流偏置") \
  X(64U, "Iv_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "V 相电流偏置") \
  X(65U, "Iw_off", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "W 相电流偏置") \
  X(80U, "p_m", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "电机位置") \
  X(81U, "xout", INFRABOT_REGISTER_F32, INFRABOT_REGISTER_READ_ONLY, 0.0f, 0.0f, INFRABOT_REGISTER_CLOSED, "输出轴位置")

typedef struct {
  uint8_t rid;
  uint8_t type;   /* INFRABOT_REGISTER_U32 / _F32 */
  uint8_t access; /* INFRABOT_REGISTER_READ_ONLY / _WRITABLE / _PROTECTED */
  uint8_t bounds; /* INFRABOT_REGISTER_CLOSED / _LOW_OPEN / _HIGH_OPEN / _OPEN */
  float low, high;
  const char *name;
  const char *description;
} InfrabotRegisterInfo;

/* The register's entry, NULL for an unknown register. */
static inline const InfrabotRegisterInfo *infrabot_register_info(uint8_t rid)
{
#define INFRABOT_REGISTER_ENTRY(r, n, t, a, lo, hi, b, d) {r, t, a, b, lo, hi, n, d},
  static const InfrabotRegisterInfo registers[] = {INFRABOT_REGISTERS(INFRABOT_REGISTER_ENTRY)};
#undef INFRABOT_REGISTER_ENTRY
  for (size_t i = 0U; i < sizeof(registers) / sizeof(registers[0]); ++i)
    if (registers[i].rid == rid) return &registers[i];
  return NULL;
}

/* 1 when raw (the register's u32 or f32 bits) is in the register's range;
 * read-only and unknown registers are never in range. */
static inline int infrabot_register_value_valid(uint8_t rid, uint32_t raw)
{
  const InfrabotRegisterInfo *info = infrabot_register_info(rid);
  if (info == NULL || info->access == INFRABOT_REGISTER_READ_ONLY) return 0;
  if (info->type == INFRABOT_REGISTER_U32)
  {
    /* Integer bounds are exact in float up to 2^24; TIMEOUT's upper bound is
     * the whole u32 range. */
    const double value = (double)raw;
    return value >= (double)info->low && value <= (double)info->high;
  }
  float value;
  memcpy(&value, &raw, sizeof(value));
  if (!(value == value) || value > INFRABOT_FLOAT_MAX || value < -INFRABOT_FLOAT_MAX) return 0;
  if ((info->bounds & INFRABOT_REGISTER_LOW_OPEN) ? value <= info->low : value < info->low) return 0;
  if ((info->bounds & INFRABOT_REGISTER_HIGH_OPEN) ? value >= info->high : value > info->high) return 0;
  return 1;
}

/* 1 when WRITE_REGISTER may write raw to rid: a writable register, in range. */
static inline int infrabot_register_write_allowed(uint8_t rid, uint32_t raw)
{
  const InfrabotRegisterInfo *info = infrabot_register_info(rid);
  return info != NULL && info->access == INFRABOT_REGISTER_WRITABLE &&
         infrabot_register_value_valid(rid, raw);
}

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* INFRABOT_PROTOCOL_H */
