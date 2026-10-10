#include "pd_board.h"
#include "pd_core.h"
#include "pd_power.h"
#include "pd_zero.h"
#include "pd_config.h"
#include <stdio.h>
#include <string.h>

PD_Channel pd_channels[2];
static PD_PIDConfig pid[2][3];
static PD_PIDState pid_state[2][3];
static volatile float shadow_u[2]; /* diagnostic calculation; NEVER a gate command */
typedef struct
{
    float reference, measured, proportional, integral, derivative, output;
} Shadow;
static volatile Shadow shadow;
static uint32_t last_shadow_log;
static uint32_t last_pin_log[2];
static uint8_t rx[524];
static uint16_t rx_used;
static uint32_t rx_at, last_heartbeat, last_telemetry, last_health_log;
static uint8_t heartbeat_seen, report_channel;
/* Once a CAN controller has appeared, UART remains a diagnostic transport
   until reset. A later UART heartbeat must never keep CAN-owned power alive. */
static uint8_t can_control_seen;
static uint32_t bad_frames;
static uint8_t reply[512];
static uint8_t tx_frame[524]; /* task context only */
static struct
{
    volatile uint8_t state;
    uint8_t channel, auto_armed;
    uint16_t points, decimation, index, divider;
    uint32_t id, hz, started;
    uint8_t data[256][20];
} trace;

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put16(uint8_t *p, uint16_t x)
{
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
}
static void put32(uint8_t *p, uint32_t x)
{
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
    p[2] = (uint8_t)(x >> 16);
    p[3] = (uint8_t)(x >> 24);
}
static float get_float(const uint8_t *p)
{
    uint32_t u = get32(p);
    float f;
    memcpy(&f, &u, 4);
    return f;
}
static void put_float(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    put32(p, u);
}
/* ===================== voltage-channel conversion (0.3.22) =================
 * Pure, unit-testable, 64-bit intermediate, round-to-nearest. Order of
 * operations: raw code -> reference correction (PD_VOLTAGE_SCALE_REF_MV) ->
 * per-channel gain -> per-channel offset. Callers must use the RESULT for
 * control, telemetry, start checks and OVP so nothing downstream re-applies a
 * correction. Current channels do NOT come through here: they keep PD_VREF_MV
 * and the pd_zero.c zero store (see pd_config.h for why).
 */
static int32_t apply_voltage_cal(int64_t mv, int32_t gain_num, int32_t gain_den, int32_t offset_mv)
{
    if (gain_den <= 0)
        return (int32_t)mv + offset_mv;
    mv = (mv * (int64_t)gain_num + (gain_den / 2)) / (int64_t)gain_den;
    return (int32_t)(mv + (int64_t)offset_mv);
}
int32_t PD_ConvertVinMv(uint16_t raw)
{
    /* VIN divider: 1098/100 = 10.98, i.e. 8.85 mV per code at 3.285 V. */
    int64_t mv = ((int64_t)raw * (int64_t)PD_VOLTAGE_SCALE_REF_MV * 1098 + (4095 * 100) / 2) /
                 (4095 * 100);
    return apply_voltage_cal(mv, PD_VIN_CAL_GAIN_NUM, PD_VIN_CAL_GAIN_DEN, PD_VIN_CAL_OFFSET_MV);
}
int32_t PD_ConvertVoutMv(uint16_t raw)
{
    /* VOUT divider: 99990/4990 = 20.038, i.e. 16.07 mV per code at 3.285 V. */
    int64_t mv = ((int64_t)raw * (int64_t)PD_VOLTAGE_SCALE_REF_MV * 99990 + (4095 * 4990LL) / 2) /
                 (4095 * 4990LL);
    return apply_voltage_cal(mv, PD_VOUT_CAL_GAIN_NUM, PD_VOUT_CAL_GAIN_DEN, PD_VOUT_CAL_OFFSET_MV);
}
static uint16_t crc16(const uint8_t *p, uint16_t n)
{
    uint16_t c = 0xffff;
    uint8_t b;
    while (n--)
    {
        c ^= (uint16_t)*p++ << 8;
        for (b = 0; b < 8; b++)
            c = (uint16_t)((c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1);
    }
    return c;
}
static int send_frame(uint8_t kind, uint16_t seq, uint8_t op, uint8_t ch, const uint8_t *p,
                       uint16_t n, uint8_t priority)
{
    tx_frame[0] = 0xa5;
    tx_frame[1] = 0x5a;
    tx_frame[2] = PD_PROTOCOL_VERSION;
    tx_frame[3] = kind;
    put16(tx_frame + 4, seq);
    tx_frame[6] = op;
    tx_frame[7] = ch;
    put16(tx_frame + 8, n);
    if (n)
        memcpy(tx_frame + 10, p, n);
    put16(tx_frame + 10 + n, crc16(tx_frame + 2, (uint16_t)(8 + n)));
    return PD_PortTransmit(tx_frame, (uint16_t)(n + 12), priority);
}
int PD_TryLog(uint8_t ch, uint8_t level, uint16_t code, const char *text)
{
    uint8_t data[PD_LOG_SIZE];
    size_t n = strlen(text);
    if (n > 96)
        n = 96;
    memset(data, 0, sizeof(data));
    put32(data, PD_PortNow());
    data[4] = level;
    put16(data + 5, code);
    memcpy(data + 7, text, n);
    return send_frame(PD_KINDS_EVENT, 0, PD_COMMANDS_LOG, ch, data, sizeof(data), code == 22);
}
void PD_Log(uint8_t ch, uint8_t level, uint16_t code, const char *text)
{
    (void)PD_TryLog(ch, level, code, text);
}
static void stop(uint8_t ch)
{
    uint32_t key = PD_PortLock();
    uint8_t i, j;
    PD_PortStop(ch);
    PD_PowerAbort(ch);
    for (i = 0; i < 2; i++)
        if (ch == 0 || ch == i + 1)
        {
            pd_channels[i].enabled = 0;
            pd_channels[i].ready = 0;
            pd_channels[i].state = pd_channels[i].fault ? PD_STATES_FAULT_LATCHED : PD_STATES_OFF;
            pd_channels[i].duty_in = 0;
            pd_channels[i].duty_out = 0;
            for (j = 0; j < 3; j++)
                PD_PIDReset(&pid_state[i][j]);
            shadow_u[i] = 0;
            if (i == 0)
                memset((void *)&shadow, 0, sizeof(shadow));
        }
    /* Captures contain only off-state samples in this release, so ordinary STOP
       need not abort them. A fault explicitly marks an in-progress trace failed. */
    PD_PortUnlock(key);
}
void PD_LatchFault(uint8_t ch, uint32_t fault)
{
    uint8_t i;
    uint32_t key = PD_PortLock();
    PD_PortFaultSnapshot(ch, fault);
    PD_PortStop(ch);
    PD_PowerAbort(ch);
    for (i = 0; i < 2; i++)
        if (ch == 0 || ch == i + 1)
        {
            pd_channels[i].fault |= fault;
            pd_channels[i].enabled = 0;
            pd_channels[i].ready = 0;
            pd_channels[i].state = PD_STATES_FAULT_LATCHED;
            pd_channels[i].duty_in = 0;
            pd_channels[i].duty_out = 0;
        }
    if (trace.state == PD_TRACE_STATES_CAPTURING && (ch == 0 || trace.channel == ch))
        trace.state = PD_TRACE_STATES_FAILED;
    PD_PortUnlock(key);
}
void PD_Init(void)
{
    uint8_t i, j;
    PD_ZeroInit();
    PD_PowerInit();
    memset(pd_channels, 0, sizeof(pd_channels));
    memset(pid, 0, sizeof(pid));
    memset(pid_state, 0, sizeof(pid_state));
    memset(&trace, 0, sizeof(trace));
    rx_used = 0;
    bad_frames = 0;
    heartbeat_seen = 0;
    can_control_seen = 0;
    report_channel = 0;
    last_heartbeat = last_telemetry = last_health_log = PD_PortNow();
    last_shadow_log = PD_PortNow();
    for (i = 0; i < 2; i++)
    {
        pd_channels[i].state = PD_STATES_OFF;
        pd_channels[i].vref_mv = 5000;
        pd_channels[i].iref_ma = 1000;
        for (j = 0; j < 3; j++)
            pid[i][j].out_max = 1.0f; /* zero gains: no guessed tuning */
    }
    stop(0);
}
/* Output window that SET_TARGET must respect.
 *
 * PD_VMIN_MV..PD_VMAX_MV (1..30 V) is the GENERIC protocol window, not what the
 * power stage can produce: CH1 is a restricted buck specified for
 * PD_BUCK_VMIN_MV..PD_BUCK_VMAX_MV and the 30 V value came from a different
 * topology. Before 0.3.20 the handler accepted the generic window, so 20 V could
 * be stored as a buck target - above the 0.9*VIN headroom, above every sane OVP
 * threshold and invisible to the operator, who only sees that the output does
 * not reach it. A command outside the window is now refused with PD_ERRORS_RANGE
 * BEFORE anything is stored, so it can neither move the ramp/reference nor raise
 * the protection threshold.
 *
 * The window is mode-INDEPENDENT on purpose: a value stored while the IL debug
 * mode is selected would still be there after switching back to NORMAL, and it
 * would then be a target this stage is not allowed to produce. */
static int32_t target_vmin_mv(uint8_t ch)
{
#if PD_POWER_STAGE_IMPLEMENTED
    if (ch == 1)
        return PD_BUCK_VMIN_MV;
#endif
    return PD_VMIN_MV;
}
static int32_t target_vmax_mv(uint8_t ch)
{
    int32_t vmax = PD_VMAX_MV;
#if PD_POWER_STAGE_IMPLEMENTED
    if (ch == 1 && PD_BUCK_VMAX_MV < vmax)
        vmax = PD_BUCK_VMAX_MV;
#endif
    return vmax;
}
static uint16_t telemetry(uint8_t ch, uint8_t *p)
{
    PD_Channel c;
    uint32_t k = PD_PortLock();
    c = pd_channels[ch - 1];
    PD_PortUnlock(k);
    put32(p, PD_PortNow());
    p[4] = c.state;
    p[5] = c.mode;
    p[6] = c.enabled;
    p[7] = c.ready;
    put32(p + 8, c.fault);
    put16(p + 12, c.valid);
    put32(p + 14, (uint32_t)c.vin_mv);
    put32(p + 18, (uint32_t)c.iin_ma);
    put32(p + 22, (uint32_t)c.vout_mv);
    put32(p + 26, (uint32_t)c.iout_ma);
    put32(p + 30, (uint32_t)c.il_ma);
    put32(p + 34, (uint32_t)c.vref_mv);
    put32(p + 38, (uint32_t)c.iref_ma);
    put32(p + 42, (uint32_t)c.ilref_ma);
    put16(p + 46, (uint16_t)c.tmos_cdeg);
    put16(p + 48, (uint16_t)c.tl_cdeg);
    put16(p + 50, c.duty_in);
    put16(p + 52, c.duty_out);
    return PD_TELEMETRY_SIZE;
}
static uint16_t pid_reply(uint8_t ch, uint8_t loop)
{
    PD_PIDConfig p;
    uint32_t k = PD_PortLock();
    p = pid[ch - 1][loop];
    PD_PortUnlock(k);
    reply[0] = loop;
    put_float(reply + 1, p.kp);
    put_float(reply + 5, p.ki);
    put_float(reply + 9, p.kd);
    put_float(reply + 13, p.tau);
    put_float(reply + 17, p.out_min);
    put_float(reply + 21, p.out_max);
    return PD_PID_SIZE;
}
static uint16_t trace_info(void)
{
    uint32_t k = PD_PortLock();
    put32(reply, trace.id);
    reply[4] = trace.state;
    put16(reply + 5, trace.points);
    put32(reply + 7, trace.hz);
    PD_PortUnlock(k);
    return PD_TRACE_INFO_SIZE;
}
static uint16_t dispatch(uint8_t op, uint8_t ch, const uint8_t *p, uint16_t n, uint16_t *out)
{
    PD_Channel *c;
    uint32_t key;
    uint8_t loop;
    int32_t v, i;
    uint16_t points, dec, count, offset;
    PD_PIDConfig new_pid;
    PD_ZeroStatus zero;
    uint16_t error;
    *out = 0;
    if (op == PD_COMMANDS_INFO || op == PD_COMMANDS_HEARTBEAT || op == PD_COMMANDS_STOP_ALL)
    {
        if (ch)
            return PD_ERRORS_BAD_CHANNEL;
        if (n)
            return PD_ERRORS_BAD_LENGTH;
        if (op == PD_COMMANDS_INFO)
        {
            memset(reply, 0, PD_INFO_SIZE);
            put32(reply, PD_SCHEMA_CRC32);
            memcpy(reply + 4, PD_FW_VERSION, sizeof(PD_FW_VERSION) - 1);
            PD_PortDeviceID((char *)reply + 36);
            put16(reply + 60, PD_COMM_TIMEOUT_MS);
            *out = PD_INFO_SIZE;
        }
        else if (op == PD_COMMANDS_HEARTBEAT)
        {
            if (!can_control_seen)
            {
                last_heartbeat = PD_PortNow();
                heartbeat_seen = 1;
            }
        }
        else
        {
            /* A deliberate STOP_ALL is how the host disconnects; do not latch a
               communication fault for the silence that follows. Outputs are
               already off, so this cannot mask a runaway stage. A later
               HEARTBEAT re-arms the watchdog. */
            heartbeat_seen = 0;
            stop(0);
        }
        return 0;
    }
    if (ch < 1 || ch > 2)
        return PD_ERRORS_BAD_CHANNEL;
    c = &pd_channels[ch - 1];
    switch (op)
    {
    case PD_COMMANDS_CAPS:
        if (n)
            return PD_ERRORS_BAD_LENGTH;
        memset(reply, 0, PD_CAPS_SIZE);
        put32(reply, PD_FEATURES_PID | PD_FEATURES_TRACE | PD_FEATURES_NOMINAL_CALIBRATION |
                         (ch == 1 ? (PD_FEATURES_NORMAL | PD_FEATURES_IL_DEBUG |
                                     PD_FEATURES_TARGET_STAGING | PD_FEATURES_ZERO_CALIBRATION)
                                  : 0));
        put32(reply + PD_CAPS_VMIN_MV_OFFSET, ch == 1 ? PD_BUCK_VMIN_MV : 0);
        put32(reply + PD_CAPS_VMAX_MV_OFFSET, ch == 1 ? PD_BUCK_VMAX_MV : 0);
        put32(reply + PD_CAPS_IMAX_MA_OFFSET, ch == 1 ? PD_IMAX_MA : 0);
        put32(reply + PD_CAPS_ILMAX_MA_OFFSET, ch == 1 ? PD_DEBUG_ILMAX_MA : 0);
        put32(reply + PD_CAPS_CURRENT_HZ_OFFSET, ch == 1 ? PD_FAST_HZ : 0);
        put32(reply + PD_CAPS_VOLTAGE_HZ_OFFSET, ch == 1 ? PD_OUTER_HZ : 0);
        put32(reply + PD_CAPS_CC_HZ_OFFSET, ch == 1 ? PD_OUTER_HZ : 0);
        put32(reply + PD_CAPS_MAX_DEBUG_MS_OFFSET, 5000);
        put16(reply + PD_CAPS_TRACE_MAX_POINTS_OFFSET, 256);
        put32(reply + PD_CAPS_TRACE_BASE_HZ_OFFSET, PD_FAST_HZ);
        *out = PD_CAPS_SIZE;
        break;
    case PD_COMMANDS_STATUS:
        if (n)
            return PD_ERRORS_BAD_LENGTH;
        *out = telemetry(ch, reply);
        break;
    case PD_COMMANDS_SET_TARGET:
        if (n != PD_TARGET_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (ch != 1)
            return PD_ERRORS_UNSUPPORTED;
        v = (int32_t)get32(p);
        i = (int32_t)get32(p + 4);
        if (v < target_vmin_mv(ch) || v > target_vmax_mv(ch) || i < 0 || i > PD_IMAX_MA)
            return PD_ERRORS_RANGE;
        /* Runtime availability check (0.3.22). The static window above is the
           CONFIGURED range (2..18 V); whether THIS input voltage can produce the
           value is a separate question, answered by the same function that gates
           START. Rules:
             - output OFF: staging anywhere inside the CAPS range is allowed, so a
               preset can be prepared before the input is connected;
             - running with fresh VIN: the value must be reachable right now;
             - VIN sample stale/invalid: do not refuse here, the loop's own
               staleness protection handles that case.
           Validate before writing, so a refused command cannot move the accepted
           target, the ramp or the descent deadline. */
        if (v > 0 && PD_PowerActive(ch) && (c->valid & PD_VALID_BITS_VIN))
        {
            int32_t vmax = PD_PowerAllowedVmaxMv(c->vin_mv);
            if (vmax <= 0 || v > vmax)
            {
                char line[128];
                (void)snprintf(line, sizeof(line),
                               "SET_TARGET %ld mV not reachable at VIN=%ld (allowed %ld mV)",
                               (long)v, (long)c->vin_mv, (long)vmax);
                PD_Log(1, 1, 14, line);
                return PD_ERRORS_RANGE;
            }
        }
        key = PD_PortLock();
        c->vref_mv = v;
        c->iref_ma = i;
        PD_PortUnlock(key);
        memcpy(reply, p, n);
        *out = n;
        break;
    case PD_COMMANDS_SET_MODE:
        if (PD_ZeroBusy())
            return PD_ERRORS_BUSY;
        if (n != 1)
            return PD_ERRORS_BAD_LENGTH;
        if (ch != 1)
            return PD_ERRORS_UNSUPPORTED;
        if (p[0] > 1)
            return PD_ERRORS_RANGE;
        if (c->state != PD_STATES_OFF)
            return PD_ERRORS_NOT_OFF;
        key = PD_PortLock();
        c->mode = p[0];
        c->ilref_ma = 0;
        PD_PIDReset(&pid_state[ch - 1][0]);
        PD_PortUnlock(key);
        reply[0] = p[0];
        *out = 1;
        break;
    case PD_COMMANDS_OUTPUT:
        if (n != 1)
            return PD_ERRORS_BAD_LENGTH;
        if (p[0] > 1)
            return PD_ERRORS_RANGE;
        if (p[0])
        {
            error = PD_PowerStart(ch);
            if (error)
                return error;
            reply[0] = 1;
        }
        else
        {
            PD_PowerRequestStop();
            reply[0] = 0;
        }
        *out = 1;
        break;
    case PD_COMMANDS_CLEAR_FAULT:
        if (n)
            return PD_ERRORS_BAD_LENGTH;
        if (c->enabled)
            return PD_ERRORS_NOT_OFF;
        if (PD_PortFaultPins(ch))
            return PD_ERRORS_FAULT;
        /* Release the diagnosis BEFORE testing for health (re-applied after the 0.3.34 revert).
           The sampling verdict used to count every error since power-up, and this command tests
           that same verdict - so once a transient had set it, CLEAR_FAULT answered NOT_READY for
           ever and the operator had to power-cycle the board. Clearing the debounce first means
           the health test below asks the fair question: is the sampler healthy NOW? If it is not,
           the test still refuses and the fault stays latched. */
        PD_BoardClearAdcFault();
        PD_BoardClearIlRun();
        if (!PD_PortSamplingHealthy())
        {
            PD_Log(ch, 2, 14, "CLEAR_FAULT refused: sampler still unhealthy");
            return PD_ERRORS_NOT_READY;
        }
        key = PD_PortLock();
        if (!PD_PortFaultPins(ch))
        {
            c->fault = 0;
            c->state = PD_STATES_OFF;
        }
        else
        {
            PD_PortUnlock(key);
            return PD_ERRORS_FAULT;
        }
        PD_PortUnlock(key);
        stop(ch);
        if (ch == 1) PD_ZeroStartupRetry();
        break;
    case PD_COMMANDS_GET_PID:
        if (n != 1)
            return PD_ERRORS_BAD_LENGTH;
        if (p[0] > 2)
            return PD_ERRORS_RANGE;
        *out = pid_reply(ch, p[0]);
        break;
    case PD_COMMANDS_SET_PID:
        if (n != PD_PID_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (c->state != PD_STATES_OFF)
            return PD_ERRORS_NOT_OFF;
        loop = p[0];
        if (loop > 2)
            return PD_ERRORS_RANGE;
        new_pid.kp = get_float(p + 1);
        new_pid.ki = get_float(p + 5);
        new_pid.kd = get_float(p + 9);
        new_pid.tau = get_float(p + 13);
        new_pid.out_min = get_float(p + 17);
        new_pid.out_max = get_float(p + 21);
        if (!PD_PIDValid(&new_pid))
            return PD_ERRORS_RANGE;
        key = PD_PortLock();
        pid[ch - 1][loop] = new_pid;
        PD_PIDReset(&pid_state[ch - 1][loop]);
        PD_PortUnlock(key);
        if (loop != 0)
            /* Be explicit: loop 1 (CV) and loop 2 (CC) are stored and read back,
               but the real voltage/current loops use the constants compiled into
               pd_config.h. Do not let the PID page imply otherwise. */
            PD_Log(ch, 1, 12, "SET_PID loop 1/2 stored only: CH1 CV/CC uses the compiled "
                              "PD_CV_*/PD_CC_* constants, not this");
        *out = pid_reply(ch, loop);
        break;
    case PD_COMMANDS_DEBUG_REF:
        if (n != PD_DEBUG_REF_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (ch != 1)
            return PD_ERRORS_UNSUPPORTED;
        if (c->state != PD_STATES_OFF || c->mode != PD_MODES_IL_DEBUG)
            return PD_ERRORS_NOT_OFF;
        i = (int32_t)get32(p);
        if (i < 0 || i > PD_DEBUG_ILMAX_MA || get32(p + 4) < 10 || get32(p + 4) > 5000)
            return PD_ERRORS_RANGE;
        key = PD_PortLock();
        c->ilref_ma = i;
        PD_PortUnlock(key);
        memcpy(reply, p, n);
        *out = n;
        break;
    case PD_COMMANDS_TRACE_ARM:
        if (PD_ZeroBusy())
            return PD_ERRORS_BUSY;
        if (n != PD_TRACE_ARM_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        points = get16(p);
        dec = get16(p + 2);
        if (points < 16 || points > 256 || dec < 1 || dec > 1000 || PD_FAST_HZ % dec)
            return PD_ERRORS_RANGE;
        if (!PD_PortSamplingHealthy() || (c->valid & PD_VALID_BITS_IL) == 0 ||
            (c->valid & PD_VALID_BITS_VOUT) == 0)
            return PD_ERRORS_NOT_READY;
        if (c->fault)
            return PD_ERRORS_FAULT;
        key = PD_PortLock();
        if (trace.state == PD_TRACE_STATES_CAPTURING)
        {
            PD_PortUnlock(key);
            return PD_ERRORS_BUSY;
        }
        trace.id++;
        trace.channel = ch;
        trace.points = points;
        trace.decimation = dec;
        trace.hz = PD_FAST_HZ / dec;
        trace.index = 0;
        trace.divider = 0;
        trace.started = PD_PortNow();
        trace.state = PD_TRACE_STATES_CAPTURING;
        PD_PortUnlock(key);
        *out = trace_info();
        break;
    case PD_COMMANDS_TRACE_INFO:
        if (n)
            return PD_ERRORS_BAD_LENGTH;
        if (!trace.id || trace.channel != ch)
            return PD_ERRORS_NOT_FOUND;
        *out = trace_info();
        break;
    case PD_COMMANDS_TRACE_READ:
        if (n != PD_TRACE_READ_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (get32(p) != trace.id || trace.channel != ch)
            return PD_ERRORS_NOT_FOUND;
        if (trace.state != PD_TRACE_STATES_COMPLETE)
            return PD_ERRORS_BUSY;
        offset = get16(p + 4);
        count = get16(p + 6);
        if (count < 1 || count > 24 || offset >= trace.points || count > trace.points - offset)
            return PD_ERRORS_RANGE;
        memcpy(reply, p, PD_TRACE_CHUNK_SIZE);
        memcpy(reply + PD_TRACE_CHUNK_SIZE, trace.data[offset], count * PD_TRACE_POINT_SIZE);
        *out = (uint16_t)(PD_TRACE_CHUNK_SIZE + count * PD_TRACE_POINT_SIZE);
        break;
    case PD_COMMANDS_ZERO:
        if (n != PD_ZERO_REQUEST_SIZE) return PD_ERRORS_BAD_LENGTH;
        if (ch != 1) return PD_ERRORS_UNSUPPORTED;
        if (p[0] > 4) return PD_ERRORS_RANGE;
        if (p[0] && trace.state == PD_TRACE_STATES_CAPTURING) return PD_ERRORS_BUSY;
        error = PD_PortZero(p[0]);
        if (error) return error;
        PD_ZeroGet(&zero);
        memset(reply, 0, PD_ZERO_STATUS_SIZE);
        reply[0] = zero.state; reply[1] = zero.sensor;
        reply[2] = zero.applied; reply[3] = zero.reason;
        put16(reply + PD_ZERO_STATUS_SAMPLES_OFFSET, zero.samples);
        put32(reply + PD_ZERO_STATUS_IOUT_UV_OFFSET, zero.iout_uv);
        put32(reply + PD_ZERO_STATUS_IL_UV_OFFSET, zero.il_uv);
        put32(reply + PD_ZERO_STATUS_PP_UV_OFFSET, zero.pp_uv);
        put32(reply + PD_ZERO_STATUS_SPAN_UV_OFFSET, zero.span_uv);
        *out = PD_ZERO_STATUS_SIZE;
        break;
    case PD_COMMANDS_GATE_TEST:
        if (n != PD_GATE_TEST_REQUEST_SIZE) return PD_ERRORS_BAD_LENGTH;
        if (ch != 1) return PD_ERRORS_UNSUPPORTED;
        *out = 0;
        return PD_GateTest(p[0], p, n, reply, out);
    default:
        return PD_ERRORS_UNSUPPORTED;
    }
    return 0;
}
/* Payload space for one reply. PD_CAPS_SIZE is 42 and PD_TELEMETRY_SIZE is 54,
   so 64 bytes is enough for every CAN answer and avoids a 512-byte copy out of
   the module-private UART frame buffer. */
static uint8_t can_reply[64];
const uint8_t *const PD_CanReply = can_reply;
static void claim_can_control(void)
{
    if (!can_control_seen)
    {
        /* Do not silently transfer an energized output from the serial host. */
        if (pd_channels[0].enabled || pd_channels[1].enabled)
            stop(0);
        can_control_seen = 1;
        PD_Log(0, 1, 4, "CAN controller connected; UART is telemetry-only");
    }
    last_heartbeat = PD_PortNow();
    heartbeat_seen = 1;
}
uint16_t PD_CanRequest(uint8_t op, uint8_t ch, const uint8_t *p, uint16_t n)
{
    /* CAN is the sole control transport after its first valid frame. The shared
       dispatcher still applies all power-stage range and fault checks. */
    uint16_t out = 0;
    if (n > sizeof(can_reply))
        return PD_ERRORS_BAD_LENGTH;
    claim_can_control();
    return dispatch(op, ch, p, n, &out);
}
void PD_CanHeartbeat(void)
{
    claim_can_control();
}
static uint8_t uart_read_only(uint8_t op)
{
    return (op == PD_COMMANDS_INFO || op == PD_COMMANDS_CAPS ||
            op == PD_COMMANDS_STATUS || op == PD_COMMANDS_GET_PID ||
            op == PD_COMMANDS_TRACE_INFO || op == PD_COMMANDS_TRACE_READ ||
            op == PD_COMMANDS_HEARTBEAT);
}
static void process(void)
{
    uint16_t length, total, result, out, seq;
    uint8_t op, ch;
    for (;;)
    {
        if (rx_used < 2)
            return;
        if (rx[0] != 0xa5 || rx[1] != 0x5a)
        {
            memmove(rx, rx + 1, --rx_used);
            continue;
        }
        if (rx_used < 10)
            return;
        length = get16(rx + 8);
        if (rx[2] != PD_PROTOCOL_VERSION || rx[3] != PD_KINDS_REQUEST || rx[7] > 2 || length > 512)
        {
            bad_frames++;
            memmove(rx, rx + 1, --rx_used);
            continue;
        }
        total = (uint16_t)(length + 12);
        if (rx_used < total)
            return;
        if (crc16(rx + 2, (uint16_t)(length + 8)) != get16(rx + 10 + length))
        {
            bad_frames++;
            memmove(rx, rx + 1, --rx_used);
            continue;
        }
        seq = get16(rx + 4);
        op = rx[6];
        ch = rx[7];
        if (seq || op == PD_COMMANDS_HEARTBEAT || op == PD_COMMANDS_STOP_ALL)
        {
            result = (can_control_seen && !uart_read_only(op)) ? PD_ERRORS_BUSY :
                     dispatch(op, ch, rx + 10, length, &out);
            if (seq)
            {
                if (result)
                {
                    put16(reply, result);
                    send_frame(PD_KINDS_NACK, seq, op, ch, reply, 2, 1);
                }
                else
                    send_frame(PD_KINDS_ACK, seq, op, ch, reply, out, 1);
            }
            if (op == PD_COMMANDS_OUTPUT && length == 1 && rx[10] == 1 && result)
                PD_Log(ch, 2, 3, "START REJECTED by precondition check; outputs remain OFF");
        }
        rx_used = (uint16_t)(rx_used - total);
        memmove(rx, rx + total, rx_used);
    }
}
void PD_Feed(const uint8_t *bytes, uint16_t n)
{
    if (rx_used && (uint32_t)(PD_PortNow() - rx_at) > 250)
    {
        rx_used = 0;
        bad_frames++;
    }
    rx_at = PD_PortNow();
    while (n--)
    {
        if (rx_used == sizeof(rx))
        {
            rx_used = 0;
            bad_frames++;
        }
        rx[rx_used++] = *bytes++;
        process();
    }
}
void PD_ResetReceiver(void)
{
    rx_used = 0;
    bad_frames++;
}
void PD_TraceAutoArm(void)
{
    uint32_t key = PD_PortLock();
    if (trace.state != PD_TRACE_STATES_CAPTURING &&
        (pd_channels[0].valid & PD_VALID_BITS_IL) != 0)
    {
        trace.id++;
        trace.channel = 1;
        trace.points = 256;
        trace.decimation = 1;
        trace.hz = PD_FAST_HZ;
        trace.index = 0;
        trace.divider = 0;
        trace.started = PD_PortNow();
        trace.auto_armed = 1;
        trace.state = PD_TRACE_STATES_CAPTURING;
    }
    PD_PortUnlock(key);
}
void PD_Poll(void)
{
    uint32_t now = PD_PortNow();
    uint8_t ch;
    uint16_t n;
    Shadow debug;
    uint32_t key;
    char line[160];
    for (ch = 1; ch <= 2; ch++)
    {
#if !PD_CH2_FITTED
        /* No CH2 board on this build. Its floating fault pins would latch SENSOR_ALERT every
           millisecond and hold CH2 in FAULT_LATCHED forever - see PD_CH2_FITTED in pd_config.h.
           Declared and logged at boot, not silent; the release gate refuses this setting. */
        if (ch == 2)
            continue;
#endif
        if (PD_PortFaultPins(ch))
        {
            uint32_t pins = PD_PortFaultPins(ch);
            /* CH2 has no power board: its fault pins float low, so report the
               first occurrence and then at most once every 30 s. */
            if ((uint32_t)(now - last_pin_log[ch - 1]) >= (ch == 2 ? 30000u : 1000u))
            {
                last_pin_log[ch - 1] = now;
                (void)snprintf(line, sizeof(line),
                               "FAULT PIN: CH%u pins=0x%02lX (bit0=OC_N low, bit1=ALERT_N low)%s",
                               ch, (unsigned long)pins,
                               ch == 2 ? " [CH2 no power board: pins float low]" : "");
                PD_Log(ch, 2, 18, line);
            }
#if !PD_OVERRIDE_NO_LATCH
            PD_LatchFault(ch, pins);
#endif
        }
    }
    if (heartbeat_seen && (uint32_t)(now - last_heartbeat) > PD_COMM_TIMEOUT_MS)
    {
        heartbeat_seen = 0;
        PD_LatchFault(0, PD_FAULTS_COMM_TIMEOUT);
        PD_Log(0, 2, 4, "HEARTBEAT TIMEOUT: outputs disabled, communication fault latched");
    }
    if ((uint32_t)(now - last_telemetry) >= 25)
    {
        last_telemetry = now;
        report_channel ^= 1;
        n = telemetry((uint8_t)(report_channel + 1), reply);
        send_frame(PD_KINDS_EVENT, 0, PD_COMMANDS_TELEMETRY, (uint8_t)(report_channel + 1), reply,
                   n, 0);
    }
    if (trace.state == PD_TRACE_STATES_CAPTURING && (uint32_t)(now - trace.started) > 3000)
        trace.state = PD_TRACE_STATES_FAILED;
    if (trace.auto_armed && trace.state != PD_TRACE_STATES_CAPTURING)
    {
        int32_t first = 0, last = 0, lo = 0, hi = 0;
        uint16_t i, count = trace.index;
        trace.auto_armed = 0;
        if (count)
        {
            first = (int32_t)get32(trace.data[0]);
            last = (int32_t)get32(trace.data[count - 1]);
            lo = hi = first;
            for (i = 1; i < count; i++)
            {
                int32_t v = (int32_t)get32(trace.data[i]);
                if (v < lo)
                    lo = v;
                if (v > hi)
                    hi = v;
            }
        }
        /* PD_LOG_VALUE returns unsigned, so the conversions are %u. The four
           milliampere values are bounded this way because this line has to fit the
           96-byte log field like every other one. */
        (void)snprintf(line, sizeof(line),
                       "PRE-TRACE n=%u/256 1st=%u lo=%u hi=%u last=%u mA (%u kHz)",
                       PD_LOG_VALUE(count), (unsigned)PD_LOG_VALUE(first),
                       (unsigned)PD_LOG_VALUE(lo), (unsigned)PD_LOG_VALUE(hi),
                       (unsigned)PD_LOG_VALUE(last), (unsigned)(PD_FAST_HZ / 1000u));
        PD_Log(1, 1, 19, line);
    }
    /* Ready reflects the live precondition set, so the UI start button cannot
       offer a start the firmware would refuse. */
    pd_channels[0].ready = PD_PowerReady(1);
    pd_channels[1].ready = 0;
    if (pd_channels[0].mode == PD_MODES_IL_DEBUG)
    {
        float request = (float)pd_channels[0].ilref_ma * 0.001f;
        float measured = (float)pd_channels[0].il_ma * 0.001f;
        if ((pd_channels[0].valid & PD_VALID_BITS_IL) && !pd_channels[0].fault)
        {
            key = PD_PortLock();
            shadow_u[0] = PD_PIDStep(&pid[0][0], &pid_state[0][0], request, measured,
                                     1.0f / (float)PD_OUTER_HZ);
            shadow.reference = request;
            shadow.measured = measured;
            shadow.proportional = pid[0][0].kp * (request - measured);
            shadow.integral = pid_state[0][0].integral;
            shadow.derivative = pid_state[0][0].derivative;
            shadow.output = shadow_u[0];
            PD_PortUnlock(key);
        }
        else
        {
            PD_PIDReset(&pid_state[0][0]);
            shadow_u[0] = 0;
            memset((void *)&shadow, 0, sizeof(shadow));
        }
    }
    if ((uint32_t)(now - last_shadow_log) >= 500 && pd_channels[0].mode == PD_MODES_IL_DEBUG)
    {
        last_shadow_log = now;
        key = PD_PortLock();
        debug = shadow;
        PD_PortUnlock(key);
        /* Low-rate display, not a substitute for fast RAM capture. */
        (void)snprintf(line, sizeof(line), "SHADOW ref=%.3g y=%.3g P=%.3g I=%.3g D=%.3g U=%.3g",
                       (double)debug.reference, (double)debug.measured, (double)debug.proportional,
                       (double)debug.integral, (double)debug.derivative, (double)debug.output);
        PD_Log(1, 0, 12, line);
    }
    if ((uint32_t)(now - last_health_log) >= 5000)
    {
        last_health_log = now;
        /* Width checked by tools/check_log_lengths.py. The calibration details are in the
           rotating code-9 CONFIG line; this one states the range, the per-channel reference
           basis and whether the calibration is applied. That last field is READ FROM
           PD_CALIBRATED rather than typed in: it used to be a literal "cal=0", which was
           correct until the calibration was applied in 0.3.24 and then contradicted the very
           next CONFIG line in the same log (0.3.26 bench: health said cal=0 while CONFIG said
           cal=1 gain=1000/1000 off=-11). The health line is the one an operator sees every
           five seconds, so a stale literal there is worse than no line at all. */
        /* EVERY number here is derived now. It used to spell out "2..24 V", "3285 mV" and
           "3300 mV" as text: the range had already been 20 V and then 24 V while this line still
           read 24, and the two references were literals that would have survived a re-calibration
           in silence. This is the line an operator reads every five seconds, so a stale literal
           here is worse than no line at all.
           Width: check_log_lengths.py counts every %ld/%u at full width, which is why the format
           is this terse. The first attempt measured 136 bytes and 98 bytes against the 95-byte
           field; "CH1 " went last because the log frame header already carries the channel, and
           "cal" is the calibration-state flag, not a voltage. */
        (void)snprintf(line, sizeof(line), "buck %ld..%ldV in %ld..%ldV ref %u/%u cal=%u",
                       (long)(PD_BUCK_VMIN_MV / 1000), (long)(PD_BUCK_VMAX_MV / 1000),
                       (long)(PD_VIN_MIN_MV / 1000), (long)(PD_VIN_MAX_MV / 1000),
                       (unsigned)PD_VOLTAGE_SCALE_REF_MV, (unsigned)PD_VREF_MV,
                       (unsigned)PD_CALIBRATED);
        PD_Log(0, 1, 1, line);
        if (bad_frames)
        {
            PD_Log(0, 1, 2,
                   "RX invalid/expired frames detected; check baud, protocol V2 and wiring");
            bad_frames = 0;
        }
    }
}
void PD_FastSample(uint8_t ch, int32_t il, int32_t vout, int32_t vin, int32_t iout, uint8_t valid)
{
    PD_Channel *c = &pd_channels[ch - 1];
    uint8_t *p;
    (void)vin;
    (void)iout;
    c->il_ma = il;
    if (valid)
        c->valid |= PD_VALID_BITS_IL;
    else
        c->valid &= (uint16_t)~PD_VALID_BITS_IL;
    /* 100 kHz ISR path stays free of float PID work.  The IL shadow PID is
       evaluated at the 1 kHz loop rate in PD_Poll instead, so the injected
       ISR keeps its headroom for the real control commit. */
    if (trace.state != PD_TRACE_STATES_CAPTURING || trace.channel != ch)
        return;
    if (!valid || (c->valid & PD_VALID_BITS_VOUT) == 0)
    {
        trace.state = PD_TRACE_STATES_FAILED;
        return;
    }
    if (trace.divider++ != 0)
    {
        if (trace.divider >= trace.decimation)
            trace.divider = 0;
        return;
    }
    if (trace.divider >= trace.decimation)
        trace.divider = 0;
    p = trace.data[trace.index];
    put32(p, (uint32_t)il);
    put32(p + 4, (uint32_t)c->ilref_ma);
    put32(p + 8, (uint32_t)vout);
    put32(p + 12, (uint32_t)c->vref_mv);
    put16(p + 16, c->duty_in);
    put16(p + 18, c->duty_out);
    trace.index++;
    if (trace.index >= trace.points)
        trace.state = PD_TRACE_STATES_COMPLETE;
}
