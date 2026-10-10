/**
  ******************************************************************************
  * @file    can_protocol.h
  * @brief   STM_POWER CAN protocol v1 - shared by BOTH MCU projects.
  *
  *          H750 (interaction / HMI board)  <-->  G474 (power board)
  *
  *          Classic CAN, standard 11-bit identifiers, 8 data bytes.
  *          Nominal bit rate 500 kbit/s.
  *
  *          THIS FILE MUST BE BYTE-IDENTICAL IN BOTH PROJECTS.
  *          Canonical copy: CODE_USE/shared/protocol/
  *
  * @note    Endianness: every multi-byte field is LITTLE-ENDIAN and is written
  *          field-by-field (see the pack/unpack helpers). Do NOT memcpy a
  *          struct onto the wire - padding and alignment are compiler-specific.
  ******************************************************************************
  */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Version / identity
 * -------------------------------------------------------------------------- */
#define CANP_PROTO_VERSION      1u      /* bump on ANY incompatible change      */

#define CANP_NODE_H750          0x01u   /* interaction board                    */
#define CANP_NODE_G474          0x02u   /* power board                          */

#define CANP_CH1                0x01u
#define CANP_CH2                0x02u

/* --------------------------------------------------------------------------
 * Message identifiers (standard 11-bit)
 * -------------------------------------------------------------------------- */
#define CANP_ID_HEARTBEAT       0x100u  /* H750 -> G474  ~250 ms                */
#define CANP_ID_SET             0x110u  /* H750 -> G474  setpoint / command     */
#define CANP_ID_SET_ACK         0x190u  /* G474 -> H750  accept / reject        */
#define CANP_ID_MEASURED        0x200u  /* G474 -> H750  periodic measurements  */
#define CANP_ID_TEMPERATURE    0x202u
#define CANP_ID_FAULT           0x201u  /* G474 -> H750  fault bitmap + inputs  */
#define CANP_ID_CAPABILITY      0x210u  /* G474 -> H750  limits / channel info  */

/* --------------------------------------------------------------------------
 * Operation codes (CANP_ID_SET byte 3)
 * -------------------------------------------------------------------------- */
#define CANP_OP_NONE            0x00u
#define CANP_OP_START           0x01u   /* output ON                            */
#define CANP_OP_STOP            0x02u   /* output OFF                           */
#define CANP_OP_SET_SETPOINT    0x03u   /* apply v_set_mv / i_limit_ma          */
#define CANP_OP_CLEAR_FAULT     0x04u   /* clear latched fault                  */
#define CANP_OP_QUERY_CAP       0x05u   /* ask G474 to (re)send 0x210           */

/* --------------------------------------------------------------------------
 * Result codes (CANP_ID_SET_ACK byte 3)
 * -------------------------------------------------------------------------- */
#define CANP_RES_ACCEPTED       0x00u
#define CANP_RES_REJECTED       0x01u   /* see error code                      */
#define CANP_RES_BUSY           0x02u   /* cannot act right now                */

/* Error codes (CANP_ID_SET_ACK byte 4) */
#define CANP_ERR_NONE           0x00u
#define CANP_ERR_RANGE_V        0x01u   /* v_set_mv outside capability         */
#define CANP_ERR_RANGE_I        0x02u   /* i_limit_ma outside capability       */
#define CANP_ERR_FAULT_LATCHED  0x03u   /* clear the fault first               */
#define CANP_ERR_CH_ABSENT      0x04u   /* channel not installed               */
#define CANP_ERR_BAD_OP         0x05u   /* unknown operation                   */
#define CANP_ERR_NOT_CALIBRATED 0x06u
/* The power board reports these three as well. The HMI needs a name for each,
 * otherwise a refusal it CAN receive would have to be shown as a generic
 * failure. Values are the power board's; see pd_can.h / pd_can_map_error(). */
#define CANP_ERR_NOT_OFF        0x07u   /* command requires the channel to be off */
#define CANP_ERR_NOT_READY      0x08u   /* preconditions unmet: calibration, gate
                                           map or healthy sampling               */
#define CANP_ERR_BAD_LENGTH     0x09u   /* malformed request for this operation   */

/* Run states (CANP_ID_MEASURED byte 3) */
#define CANP_STATE_OFF          0x00u
#define CANP_STATE_CV           0x01u   /* constant voltage                     */
#define CANP_STATE_CC           0x02u   /* constant current                     */
#define CANP_STATE_FAULT        0x03u
#define CANP_STATE_ABSENT       0x04u   /* channel not installed                */
#define CANP_STATE_STARTING     0x05u   /* precharge/soft start, output enabled */
#define CANP_STATE_STOPPING     0x06u   /* commanded stop ramp still active     */

/* Fault bits (CANP_ID_FAULT bytes 2..5, little-endian uint32) */
#define CANP_FAULT_OVP          0x00000001u  /* output over-voltage             */
#define CANP_FAULT_OCP          0x00000002u  /* output over-current             */
#define CANP_FAULT_OTP          0x00000004u  /* over-temperature                */
#define CANP_FAULT_OPP          0x00000008u  /* over-power                      */
#define CANP_FAULT_UVLO         0x00000010u  /* input under-voltage             */
#define CANP_FAULT_REVERSE      0x00000020u  /* reverse polarity                */
#define CANP_FAULT_ALERT_N      0x00000040u  /* hardware ALERT_N asserted       */
#define CANP_FAULT_CAN_TIMEOUT  0x00000080u  /* host heartbeat lost (G474 side) */
#define CANP_FAULT_SENSOR       0x00000100u  /* implausible measurement (board: ADC)    */
#define CANP_FAULT_INTERNAL     0x00000200u  /* board-internal fault (board: INTERNAL)  */

/* Flags (CANP_ID_HEARTBEAT byte 3) */
#define CANP_FLAG_OUTPUT_ON     0x01u
#define CANP_FLAG_FAULT_LATCHED 0x02u
#define CANP_FLAG_CH2_PRESENT   0x04u

/* Capability flags (CANP_ID_CAPABILITY byte 3) */
#define CANP_CAP_CH_PRESENT     0x01u
#define CANP_CAP_CV_CC          0x02u
#define CANP_CAP_REMOTE_CLEAR   0x04u
#define CANP_CAP_READY          0x08u   /* current start precheck passes */
#define CANP_CAP_READY_KNOWN    0x10u   /* sender supports the READY flag */
#define CANP_CAP_INIT_FAILED    0x20u   /* startup IOUT calibration failed */

/* --------------------------------------------------------------------------
 * Timeouts / periods (milliseconds)
 * -------------------------------------------------------------------------- */
#define CANP_HEARTBEAT_PERIOD_MS   250u
#define CANP_TELEMETRY_PERIOD_MS   100u
/* G474 MUST stop the output if no heartbeat arrives within this window. */
#define CANP_HEARTBEAT_TIMEOUT_MS  1000u
/* H750 shows "device offline" after this long with no 0x200/0x201. */
#define CANP_TEMP_STALE_MS         1000u
#define CANP_LINK_LOST_MS          500u

/* --------------------------------------------------------------------------
 * Message lengths used on the wire (bytes actually transmitted)
 * -------------------------------------------------------------------------- */
#define CANP_LEN_HEARTBEAT      4u
#define CANP_LEN_SET            8u
#define CANP_LEN_SET_ACK        8u
#define CANP_LEN_MEASURED       8u
#define CANP_LEN_TEMPERATURE   8u
#define CANP_LEN_FAULT          8u
#define CANP_LEN_CAPABILITY     8u

/* --------------------------------------------------------------------------
 * Little-endian helpers (kept explicit so both toolchains agree)
 * -------------------------------------------------------------------------- */
static inline void canp_put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline uint16_t canp_get_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void canp_put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline uint32_t canp_get_u32(const uint8_t *p)
{
  return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* --------------------------------------------------------------------------
 * Payload layouts
 * -------------------------------------------------------------------------- */

/* 0x100 H750 -> G474, 4 bytes, every CANP_HEARTBEAT_PERIOD_MS
 *   [0] proto version
 *   [1] sender node id
 *   [2] sequence (wraps at 256)
 *   [3] flags (CANP_FLAG_*)                                                */
#define CANP_HB_VER        0u
#define CANP_HB_NODE       1u
#define CANP_HB_SEQ        2u
#define CANP_HB_FLAGS      3u

/* 0x110 H750 -> G474, 8 bytes
 *   [0..1] sequence          (u16, echoed in the ACK)
 *   [2]    channel           (CANP_CH1 / CANP_CH2)
 *   [3]    operation         (CANP_OP_*)
 *   [4..5] target voltage    (u16, millivolts)
 *   [6..7] current limit     (u16, milliamps)
 * For CANP_OP_START / _STOP / _CLEAR_FAULT the value fields are ignored.   */
#define CANP_SET_SEQ       0u
#define CANP_SET_CH        2u
#define CANP_SET_OP        3u
#define CANP_SET_V_MV      4u
#define CANP_SET_I_MA      6u

/* 0x190 G474 -> H750, 8 bytes
 *   [0..1] sequence          (echo of the request)
 *   [2]    channel
 *   [3]    result            (CANP_RES_*)
 *   [4]    error code        (CANP_ERR_*)
 *   [5..6] applied voltage   (u16 mV, the value the G474 actually accepted)
 *   [7]    run state after handling (CANP_STATE_*)                         */
#define CANP_ACK_SEQ       0u
#define CANP_ACK_CH        2u
#define CANP_ACK_RES       3u
#define CANP_ACK_ERR       4u
#define CANP_ACK_V_MV      5u
#define CANP_ACK_STATE     7u

/* 0x200 G474 -> H750, 8 bytes, every CANP_TELEMETRY_PERIOD_MS
 *   [0..1] output voltage    (u16, millivolts)
 *   [2..3] output current    (u16, milliamps)
 *   [4]    channel
 *   [5]    run state         (CANP_STATE_*)
 *   [6]    fault summary     (0 = clear, else low byte of the fault bitmap)
 *   [7]    sequence (wraps)                                                */
#define CANP_MS_V_MV       0u
#define CANP_MS_I_MA       2u
#define CANP_MS_CH         4u
#define CANP_MS_STATE      5u
#define CANP_MS_FAULT_LO   6u
#define CANP_MS_SEQ        7u

/* 0x201 G474 -> H750, 8 bytes, sent immediately when a fault changes
 *   [0]    channel
 *   [1]    flags (CANP_FLAG_*)
 *   [2..5] fault bitmap      (u32, CANP_FAULT_*)
 *   [6..7] input voltage     (u16, millivolts)                            */
#define CANP_FLT_CH        0u
#define CANP_FLT_FLAGS     1u
#define CANP_FLT_BITS      2u
#define CANP_FLT_VIN_MV    6u

/* 0x202 G474 -> H750, 8 bytes, every 100 ms, independent of output state.
 * [0..1] MOS temperature, [2..3] inductor temperature: signed int16, 0.01 C.
 * [4] channel; [5] valid bits (bit0 MOS, bit1 inductor); [6] sequence; [7] 0.
 * Invalid = -32768 (0x8000) AND valid bit clear. A sensor error never means 0 C.
 * This optional v1 extension does not refresh the main link/output watchdog. */
#define CANP_TEMP_MOS_CDEG  0u
#define CANP_TEMP_L_CDEG    2u
#define CANP_TEMP_CH        4u
#define CANP_TEMP_VALID     5u
#define CANP_TEMP_SEQ       6u
#define CANP_TEMP_MOS_OK    0x01u
#define CANP_TEMP_L_OK      0x02u
#define CANP_TEMP_INVALID   (-32768)

/* 0x210 G474 -> H750, 8 bytes, answered to CANP_OP_QUERY_CAP and at link-up
 *   [0]    channel
 *   [1..2] minimum settable voltage (u16, millivolts)
 *   [3..4] maximum settable voltage (u16, millivolts)
 *   [5..6] maximum current limit    (u16, milliamps; 0 = unknown)
 *   [7]    capability flags  (CANP_CAP_*)
 *
 * The H750 MUST take its UI limits from here, never from compiled-in
 * constants, so that a power-board change does not require an HMI reflash. */
#define CANP_CAP_CH        0u
#define CANP_CAP_VMIN_MV   1u
#define CANP_CAP_VMAX_MV   3u
#define CANP_CAP_IMAX_MA   5u
#define CANP_CAP_FLAGS     7u

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
