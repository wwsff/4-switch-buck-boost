/* Standalone check for App/pd_can.c - the CAN wire format.
 *
 * Kept separate from tools/test_native.py on purpose: that suite currently has
 * pre-existing failures, and a new check buried among them would be invisible.
 * Run:  tools/run_can_codec_test.py
 *
 * It asserts the EXACT bytes on the wire, because the HMI board hard-codes the
 * same offsets. If one side drifts, this test or its counterpart on the H750
 * must fail loudly rather than the two boards silently disagreeing.
 */
#include "pd_can.h"
#include <stdio.h>
#include <string.h>

static int failures;
static int checks;

static void ok(int cond, const char *what)
{
    checks++;
    if (!cond)
    {
        failures++;
        printf("  FAIL %s\n", what);
    }
}

static void eq_bytes(const uint8_t *got, const uint8_t *want, int n, const char *what)
{
    checks++;
    if (memcmp(got, want, (size_t)n) != 0)
    {
        int i;
        failures++;
        printf("  FAIL %s\n    got ", what);
        for (i = 0; i < n; i++)
            printf("%02X ", got[i]);
        printf("\n    want ");
        for (i = 0; i < n; i++)
            printf("%02X ", want[i]);
        printf("\n");
    }
}

int main(void)
{
    uint8_t d[8];
    pd_can_request r;

    printf("pd_can codec checks\n");

    /* ---- decode: heartbeat 0x100 ---- */
    {
        uint8_t hb[4] = {1u, 1u, 7u, 0x01u};
        memset(&r, 0, sizeof(r));
        ok(pd_can_decode(0x100u, hb, 4u, &r) == 1, "heartbeat accepted");
        ok(r.kind == PD_CAN_KIND_HEARTBEAT, "heartbeat kind");
        ok(r.flags == 0x01u, "heartbeat flags carried");
        ok(pd_can_decode(0x100u, hb, 3u, &r) == 0, "short heartbeat rejected");
    }

    /* ---- decode: command 0x110, setpoint 5000 mV / 1000 mA ---- */
    {
        uint8_t cmd[8] = {0x11u, 0x00u, 0x01u, 0x03u, 0x88u, 0x13u, 0xE8u, 0x03u};
        /* seq=0x0011, ch=1, op=SET, v=0x1388=5000, i=0x03E8=1000 */
        memset(&r, 0, sizeof(r));
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 1, "setpoint accepted");
        ok(r.seq == 0x0011u, "seq little-endian");
        ok(r.channel == 1u, "channel");
        ok(r.can_op == 0x03u, "can op");
        ok(r.op == PD_COMMANDS_SET_TARGET, "op maps to SET_TARGET");
        ok(r.v_mv == 5000u, "v_mv little-endian");
        ok(r.i_ma == 1000u, "i_ma little-endian");
    }

    /* ---- decode: operation mapping ---- */
    {
        uint8_t cmd[8] = {0x01u, 0x00u, 0x01u, 0x01u, 0, 0, 0, 0};
        memset(&r, 0, sizeof(r));
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 1, "start accepted");
        ok(r.op == PD_COMMANDS_OUTPUT, "START maps to OUTPUT");
        cmd[3] = 0x02u;
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 1, "stop accepted");
        ok(r.op == PD_COMMANDS_OUTPUT, "STOP maps to OUTPUT");
        cmd[3] = 0x04u;
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 1, "clear accepted");
        ok(r.op == PD_COMMANDS_CLEAR_FAULT, "CLEAR maps to CLEAR_FAULT");
        cmd[3] = 0x05u;
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 1, "query accepted");
        ok(r.op == PD_COMMANDS_CAPS, "QUERY_CAP maps to CAPS");
        cmd[3] = 0x7Fu;
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 0, "unknown op rejected");
        ok(pd_can_map_op(0x7Fu) == PD_CAN_OP_INVALID, "unknown op maps to INVALID");
    }

    /* ---- decode: rejections ---- */
    {
        uint8_t cmd[8] = {0x01u, 0x00u, 0x00u, 0x01u, 0, 0, 0, 0}; /* ch 0 */
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 0, "channel 0 rejected");
        cmd[2] = 3u;
        ok(pd_can_decode(0x110u, cmd, 8u, &r) == 0, "channel 3 rejected");
        cmd[2] = 1u;
        ok(pd_can_decode(0x110u, cmd, 7u, &r) == 0, "dlc 7 rejected");
        ok(pd_can_decode(0x2FFu, cmd, 8u, &r) == 0, "unknown id rejected");
    }

    /* ---- encode: result 0x190, accepted start ---- */
    {
        uint8_t want[8] = {0x11u, 0x00u, 0x01u, 0x00u, 0x00u, 0x88u, 0x13u, 0x01u};
        /* seq=0x0011, ch=1, RESULT=ACCEPTED(0), ERR=NONE(0), v=5000, state=CV(1) */
        uint8_t n = pd_can_encode_result(0x0011u, 1u, 0u, 5000u, 0x01u, d);
        ok(n == 8u, "result dlc 8");
        eq_bytes(d, want, 8, "result frame bytes");
    }

    /* ---- encode: result 0x190, rejected with RANGE ---- */
    {
        uint8_t want[8] = {0x22u, 0x00u, 0x01u, 0x01u, 0x01u, 0x00u, 0x00u, 0x00u};
        uint8_t n = pd_can_encode_result(0x0022u, 1u, PD_ERRORS_RANGE, 0u, 0x00u, d);
        ok(n == 8u, "nack dlc 8");
        eq_bytes(d, want, 8, "nack frame bytes");
    }

    /* ---- error mapping: every core error lands on a defined HMI code ---- */
    {
        uint8_t res = 0xEEu, e;
        e = pd_can_map_error(0u, &res);
        ok(e == 0u && res == PD_CAN_RES_ACCEPTED, "ok -> accepted");
        e = pd_can_map_error(PD_ERRORS_RANGE, &res);
        ok(e == PD_CAN_ERR_RANGE && res == PD_CAN_RES_REJECTED, "range -> RANGE");
        e = pd_can_map_error(PD_ERRORS_NOT_OFF, &res);
        ok(e == PD_CAN_ERR_NOT_OFF, "not_off -> NOT_OFF");
        e = pd_can_map_error(PD_ERRORS_FAULT, &res);
        ok(e == PD_CAN_ERR_FAULT_LATCHED, "fault -> FAULT_LATCHED");
        e = pd_can_map_error(PD_ERRORS_BUSY, &res);
        ok(e == 0u && res == PD_CAN_RES_BUSY, "busy -> BUSY, no error");
        e = pd_can_map_error(PD_ERRORS_NOT_READY, &res);
        ok(e == PD_CAN_ERR_NOT_READY, "not_ready -> NOT_READY");
    }

    /* ---- fault mapping: a core bit must never vanish ---- */
    {
        ok(pd_can_map_fault(0u) == 0u, "no fault -> 0");
        ok(pd_can_map_fault(PD_FAULTS_OVERVOLTAGE) == PD_CAN_FAULT_OVP, "OVP");
        ok(pd_can_map_fault(PD_FAULTS_OVERCURRENT) == PD_CAN_FAULT_OCP, "OCP");
        ok(pd_can_map_fault(PD_FAULTS_OVERTEMPERATURE) == PD_CAN_FAULT_OTP, "OTP");
        ok(pd_can_map_fault(PD_FAULTS_COMM_TIMEOUT) == PD_CAN_FAULT_CAN_TIMEOUT, "COMM");
        ok((pd_can_map_fault(PD_FAULTS_OVERVOLTAGE | PD_FAULTS_OVERCURRENT) ==
            (PD_CAN_FAULT_OVP | PD_CAN_FAULT_OCP)),
           "two faults combine");
        /* every defined core fault bit must map to something non-zero */
        ok(pd_can_map_fault(PD_FAULTS_SENSOR_ALERT) != 0u, "SENSOR_ALERT mapped");
        ok(pd_can_map_fault(PD_FAULTS_ADC) != 0u, "ADC mapped");
        ok(pd_can_map_fault(PD_FAULTS_INPUT) != 0u, "INPUT mapped");
        ok(pd_can_map_fault(PD_FAULTS_INTERNAL) != 0u, "INTERNAL mapped");
    }

    /* ---- state mapping: the HMI must not show "on" while booting ---- */
    {
        ok(pd_can_map_state(PD_STATES_RUN_CV, 1u) == 0x01u, "RUN_CV -> CV");
        ok(pd_can_map_state(PD_STATES_RUN_CC, 1u) == 0x02u, "RUN_CC -> CC");
        ok(pd_can_map_state(PD_STATES_OFF, 0u) == 0x00u, "OFF -> OFF");
        ok(pd_can_map_state(PD_STATES_FAULT_LATCHED, 0u) == 0x03u, "FAULT -> FAULT");
    ok(pd_can_map_state(PD_STATES_SOFT_START, 1u) == 0x05u, "SOFT_START -> STARTING");
    ok(pd_can_map_state(PD_STATES_BOOTSTRAP_PRECHARGE, 1u) == 0x05u, "PRECHARGE -> STARTING");
    ok(pd_can_map_state(PD_STATES_STOPPING, 1u) == 0x06u, "active stop -> STOPPING");
    ok(pd_can_map_state(PD_STATES_SOFT_START, 0u) == 0x00u, "disabled soft start -> OFF");
        ok(pd_can_map_state(PD_STATES_BOOT, 0u) == 0x00u, "BOOT -> OFF");
    }

    /* ---- encode: measured 0x200 ---- */
    {
        uint8_t want[8] = {0x88u, 0x13u, 0xE8u, 0x03u, 0x01u, 0x02u, 0x02u, 0x05u};
        /* v=5000, i=1000, ch=1, state=CC(2), fault low byte=0x02, seq=5 */
        uint8_t n = pd_can_encode_measured(5000u, 1000u, 1u, 0x02u, 0x02u, 5u, d);
        ok(n == 8u, "measured dlc 8");
        eq_bytes(d, want, 8, "measured frame bytes");
    }

    /* ---- encode: fault 0x201 ---- */
    {
        uint8_t want[8] = {0x01u, 0x03u, 0x04u, 0x00u, 0x00u, 0x00u, 0x10u, 0x27u};
        /* ch=1, flags=OUTPUT_ON|FAULT_LATCHED=0x03, fault=4 (OTP),
           vin=0x2710=10000 */
        uint8_t n = pd_can_encode_fault(1u, PD_CAN_FAULT_OTP, 10000u, 1u, d);
        ok(n == 8u, "fault dlc 8");
        eq_bytes(d, want, 8, "fault frame bytes");
    }

    /* ---- encode: capability 0x210 ---- */
    {
        uint8_t want[8] = {0x01u, 0xD0u, 0x07u, 0xF0u, 0x0Au, 0x88u, 0x13u, 0x07u};
        /* ch=1, vmin=0x07D0=2000, vmax=0x0AF0=2800, imax=0x1388=5000,
           flags=PRESENT|CV_CC|REMOTE_CLEAR=0x07 */
        uint8_t n = pd_can_encode_capability(1u, 2000u, 2800u, 5000u, 1u, d);
        ok(n == 8u, "capability dlc 8");
        eq_bytes(d, want, 8, "capability frame bytes");
        /* absent channel advertises zero range and no present flag */
        n = pd_can_encode_capability(2u, 0u, 0u, 0u, 0u, d);
        ok(n == 8u, "absent capability dlc 8");
        ok(d[7] == 0x06u, "absent channel clears PRESENT but keeps CV_CC|CLEAR");
    }

    /* ---- round trip: a setpoint the HMI sends must survive decode ---- */
    {
        uint8_t cmd[8];
        pd_can_request rr;
        cmd[0] = 0x2Au;
        cmd[1] = 0x00u;
        cmd[2] = 0x01u;
        cmd[3] = 0x03u;
        cmd[4] = 0x40u; /* 0x1F40 = 8000 mV */
        cmd[5] = 0x1Fu;
        cmd[6] = 0x64u; /* 0x0064 = 100 mA */
        cmd[7] = 0x00u;
        ok(pd_can_decode(0x110u, cmd, 8u, &rr) == 1, "round trip decode");
        ok(rr.seq == 42u && rr.v_mv == 8000u && rr.i_ma == 100u, "round trip values");
    }

    {
        uint8_t want[8] = {0xC4, 0x09, 0x0C, 0xFE, 1, 3, 255, 0};
        ok(pd_can_encode_temperature(2500, -500, 1, 3, 255, d) == 8, "temperature DLC");
        eq_bytes(d, want, 8, "signed temperatures exact bytes");
        pd_can_encode_temperature(2500, 0, 1, 1, 0, d);
        ok(d[2] == 0 && d[3] == 0x80 && d[5] == 1, "invalid coil is sentinel not zero");
        pd_can_encode_temperature(0, 0, 1, 0xFC, 0, d);
        ok(d[1] == 0x80 && d[3] == 0x80 && d[5] == 0, "valid flags limited to sensors");
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
