/* CAN transport for the power board: pure encode/decode.
 *
 * This file must stay free of HAL and FreeRTOS so tools/compile_variants.py can
 * compile it with -Wall -Wextra -Werror on the host, and so the field layout can
 * be reviewed on its own. Everything that touches FDCAN2 lives in pd_can_port.c.
 *
 * WIRE FORMAT - agreed with the HMI board, see can_protocol.h there. Standard
 * 11-bit identifiers, classic CAN, 8 data bytes, every multi-byte field
 * LITTLE-ENDIAN. Do not change an offset without changing both projects.
 *
 *   0x100 HMI  -> power  heartbeat      ver, node, seq, flags
 *   0x110 HMI  -> power  command        seq(2), ch, op, v_mv(2), i_ma(2)
 *   0x190 power-> HMI    command result seq(2), ch, result, err, v_mv(2), state
 *   0x200 power-> HMI    measurement    v_mv(2), i_ma(2), ch, state, fault_lo, seq
 *   0x201 power-> HMI    fault          ch, flags, fault(4), vin_mv(2)
 *   0x210 power-> HMI    capability     ch, vmin(2), vmax(2), imax(2), flags
 */
#include "pd_can.h"

/* Identifiers -------------------------------------------------------------- */
#define PD_CAN_ID_HEARTBEAT 0x100u
#define PD_CAN_ID_COMMAND 0x110u
#define PD_CAN_ID_RESULT 0x190u
#define PD_CAN_ID_MEASURED 0x200u
#define PD_CAN_ID_FAULT 0x201u
#define PD_CAN_ID_CAPABILITY 0x210u

/* Operations, as sent by the HMI (0x110 byte 3). These are the CAN-side codes;
 * pd_can_map_op() turns them into the PD_COMMANDS_* the core understands. */
#define PD_CAN_OP_START 0x01u
#define PD_CAN_OP_STOP 0x02u
#define PD_CAN_OP_SET_SETPOINT 0x03u
#define PD_CAN_OP_CLEAR_FAULT 0x04u
#define PD_CAN_OP_QUERY_CAP 0x05u

/* Run states reported to the HMI (0x200 byte 5) */
#define PD_CAN_STATE_OFF 0x00u
#define PD_CAN_STATE_CV 0x01u
#define PD_CAN_STATE_CC 0x02u
#define PD_CAN_STATE_FAULT 0x03u
#define PD_CAN_STATE_ABSENT 0x04u
#define PD_CAN_STATE_STARTING 0x05u
#define PD_CAN_STATE_STOPPING 0x06u

#define PD_CAN_DLC 8u
#define PD_CAN_PROTO_VERSION 1u /* shared can_protocol.h: CANP_PROTO_VERSION */
#define PD_CAN_H750_NODE 1u     /* shared can_protocol.h: CANP_NODE_H750 */

/* Byte order helpers - explicit, because the two toolchains must agree. */
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}
static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

/* ---------------------------------------------------------------------------
 * Operation mapping
 * ------------------------------------------------------------------------- */
uint8_t pd_can_map_op(uint8_t can_op)
{
    switch (can_op)
    {
    case PD_CAN_OP_START:
        return PD_COMMANDS_OUTPUT;
    case PD_CAN_OP_STOP:
        return PD_COMMANDS_OUTPUT;
    case PD_CAN_OP_SET_SETPOINT:
        return PD_COMMANDS_SET_TARGET;
    case PD_CAN_OP_CLEAR_FAULT:
        return PD_COMMANDS_CLEAR_FAULT;
    case PD_CAN_OP_QUERY_CAP:
        return PD_COMMANDS_CAPS;
    default:
        return PD_CAN_OP_INVALID; /* not an operation this board implements */
    }
}

/* The core's error codes and the HMI's are different numbering spaces; this is
 * the only place that maps between them. Anything unmapped becomes REJECTED with
 * a generic error so the HMI still shows a refusal rather than a success. */
uint8_t pd_can_map_error(uint16_t pd_error, uint8_t *can_result)
{
    switch (pd_error)
    {
    case 0u:
        *can_result = PD_CAN_RES_ACCEPTED;
        return 0u; /* no error */
    case PD_ERRORS_BAD_LENGTH:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_BAD_LENGTH;
    case PD_ERRORS_BAD_CHANNEL:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_CH_ABSENT;
    case PD_ERRORS_UNSUPPORTED:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_BAD_OP;
    case PD_ERRORS_RANGE:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_RANGE;
    case PD_ERRORS_NOT_OFF:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_NOT_OFF;
    case PD_ERRORS_NOT_READY:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_NOT_READY;
    case PD_ERRORS_FAULT:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_FAULT_LATCHED;
    case PD_ERRORS_BUSY:
        *can_result = PD_CAN_RES_BUSY;
        return 0u;
    default:
        *can_result = PD_CAN_RES_REJECTED;
        return PD_CAN_ERR_FAULT_LATCHED;
    }
}

/* Fault bitmap (core PD_FAULTS_*) -> HMI CANP_FAULT_* bits */
uint32_t pd_can_map_fault(uint32_t pd_fault)
{
    uint32_t out = 0u;
    if (pd_fault & PD_FAULTS_OVERVOLTAGE)
        out |= PD_CAN_FAULT_OVP;
    if (pd_fault & PD_FAULTS_OVERCURRENT)
        out |= PD_CAN_FAULT_OCP;
    if (pd_fault & PD_FAULTS_OVERTEMPERATURE)
        out |= PD_CAN_FAULT_OTP;
    if (pd_fault & PD_FAULTS_SENSOR_ALERT)
        out |= PD_CAN_FAULT_ALERT_N;
    if (pd_fault & PD_FAULTS_ADC)
        out |= PD_CAN_FAULT_ADC;
    if (pd_fault & PD_FAULTS_INPUT)
        out |= PD_CAN_FAULT_UVLO;
    if (pd_fault & PD_FAULTS_COMM_TIMEOUT)
        out |= PD_CAN_FAULT_CAN_TIMEOUT;
    if (pd_fault & PD_FAULTS_INTERNAL)
        out |= PD_CAN_FAULT_INTERNAL;
    return out;
}

/* Core state -> the state the HMI displays */
uint8_t pd_can_map_state(uint8_t pd_state, uint8_t enabled)
{
    switch (pd_state)
    {
    case PD_STATES_RUN_CV:
        return PD_CAN_STATE_CV;
    case PD_STATES_RUN_CC:
        return PD_CAN_STATE_CC;
    case PD_STATES_FAULT_LATCHED:
        return PD_CAN_STATE_FAULT;
    case PD_STATES_STOPPING:
        return enabled ? PD_CAN_STATE_STOPPING : PD_CAN_STATE_OFF;
    case PD_STATES_BOOTSTRAP_PRECHARGE:
    case PD_STATES_SOFT_START:
        return enabled ? PD_CAN_STATE_STARTING : PD_CAN_STATE_OFF;
    case PD_STATES_OFF:
        return PD_CAN_STATE_OFF;
    default:
        /* Other boot/self-test states have not enabled the output. */
        return enabled ? PD_CAN_STATE_OFF : PD_CAN_STATE_OFF;
    }
}

/* ---------------------------------------------------------------------------
 * Decode
 * ------------------------------------------------------------------------- */
int pd_can_decode(uint32_t id, const uint8_t *data, uint8_t dlc, pd_can_request *out)
{
    if (data == 0 || out == 0)
        return 0;

    if (id == PD_CAN_ID_HEARTBEAT)
    {
        if (dlc < 4u || data[0] != PD_CAN_PROTO_VERSION || data[1] != PD_CAN_H750_NODE)
            return 0;
        out->kind = PD_CAN_KIND_HEARTBEAT;
        out->seq = 0u;
        out->channel = 0u;
        out->op = 0u;
        out->v_mv = 0u;
        out->i_ma = 0u;
        out->flags = data[3];
        return 1;
    }

    if (id == PD_CAN_ID_COMMAND)
    {
        if (dlc < PD_CAN_DLC)
            return 0;
        out->kind = PD_CAN_KIND_COMMAND;
        out->seq = get16(data + 0);
        out->channel = data[2];
        out->can_op = data[3];
        out->v_mv = get16(data + 4);
        out->i_ma = get16(data + 6);
        out->op = pd_can_map_op(out->can_op);
        out->flags = 0u;
        if (out->channel < 1u || out->channel > 2u)
            return 0;
        switch (out->can_op)
        {
        case PD_CAN_OP_START:
        case PD_CAN_OP_STOP:
        case PD_CAN_OP_CLEAR_FAULT:
        case PD_CAN_OP_QUERY_CAP:
        case PD_CAN_OP_SET_SETPOINT:
            return 1;
        default:
            return 0;
        }
    }

    return 0; /* not a frame the power board accepts */
}

/* ---------------------------------------------------------------------------
 * Encode
 * ------------------------------------------------------------------------- */
uint8_t pd_can_encode_result(uint16_t seq, uint8_t channel, uint16_t pd_error,
                             uint16_t applied_v_mv, uint8_t state, uint8_t *out)
{
    uint8_t result;
    uint8_t err = pd_can_map_error(pd_error, &result);

    put16(out + 0, seq);
    out[2] = channel;
    out[3] = result;
    out[4] = err;
    put16(out + 5, applied_v_mv);
    out[7] = state;
    return PD_CAN_DLC;
}

uint8_t pd_can_encode_measured(uint16_t v_mv, uint16_t i_ma, uint8_t channel, uint8_t state,
                               uint32_t fault, uint8_t seq, uint8_t *out)
{
    put16(out + 0, v_mv);
    put16(out + 2, i_ma);
    out[4] = channel;
    out[5] = state;
    /* Only the low byte of the fault bitmap fits; 0x201 carries the full word. */
    out[6] = (uint8_t)(fault & 0xffu);
    out[7] = seq;
    return PD_CAN_DLC;
}

uint8_t pd_can_encode_fault(uint8_t channel, uint32_t fault, uint16_t vin_mv, uint8_t output_on,
                            uint8_t *out)
{
    out[0] = channel;
    out[1] = (uint8_t)(output_on ? PD_CAN_FLAG_OUTPUT_ON : 0u);
    if (fault != 0u)
        out[1] |= PD_CAN_FLAG_FAULT_LATCHED;
    put32(out + 2, fault);
    put16(out + 6, vin_mv);
    return PD_CAN_DLC;
}

uint8_t pd_can_encode_capability(uint8_t channel, uint16_t vmin_mv, uint16_t vmax_mv,
                                 uint16_t imax_ma, uint8_t present, uint8_t *out)
{
    out[0] = channel;
    put16(out + 1, vmin_mv);
    put16(out + 3, vmax_mv);
    put16(out + 5, imax_ma);
    out[7] = (uint8_t)((present ? PD_CAN_CAP_CH_PRESENT : 0u) | PD_CAN_CAP_CV_CC |
                       PD_CAN_CAP_REMOTE_CLEAR);
    return PD_CAN_DLC;
}

/* Signed temperatures use explicit two's-complement bytes, including -32768. */
uint8_t pd_can_encode_temperature(int16_t mos, int16_t coil, uint8_t channel,
                                  uint8_t valid, uint8_t seq, uint8_t *out)
{
    valid &= PD_CAN_TEMP_MOS_OK | PD_CAN_TEMP_L_OK;
    put16(out + 0, (uint16_t)((valid & PD_CAN_TEMP_MOS_OK) ? mos : PD_CAN_TEMP_INVALID));
    put16(out + 2, (uint16_t)((valid & PD_CAN_TEMP_L_OK) ? coil : PD_CAN_TEMP_INVALID));
    out[4] = channel;
    out[5] = valid;
    out[6] = seq;
    out[7] = 0u;
    return 8u;
}
