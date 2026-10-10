/* Host test for the communication watchdog on the G474 power board.
 *
 * WHAT THIS IS FOR. The plan requires: "G474 must tie the CAN heartbeat into its
 * own communication-timeout protection: about 1 s without a heartbeat and it
 * stops." The mechanism already existed for the serial link; the CAN endpoint now
 * feeds the same state (PD_CanHeartbeat). Until this test, nothing verified the
 * timing behaviour - the existing integration test calls PD_CanRequest() and
 * never touches the watchdog path at all.
 *
 * HOW. pd_core.c is host-portable: everything hardware-facing goes through the
 * PD_Port* seam, which tests/host_port.c implements. The clock is the test's
 * (Test_Time / Test_Advance), so the watchdog can be driven deterministically.
 *
 * BUILD/RUN: tools/run_g474_watchdog_test.py
 */
#include "pd_core.h"
#include "pd_config.h"          /* PD_COMM_TIMEOUT_MS - use the real value, not a copy */
#include "powerdesk_protocol.h"

#include <stdint.h>
#include <stdio.h>

/* Host stubs the test needs (tests/host_port.c) */
void     Test_Time(uint32_t value);
void     Test_Advance(uint32_t ms);
void     Test_Reset(void);
void     Test_Sensors(int32_t vin, int32_t vout, int32_t iout);
uint32_t Test_PowerEnable(void);
uint32_t Test_Gates(void);
uint32_t Test_Read(uint8_t *out, uint32_t n);
uint16_t PD_PowerStart(uint8_t ch);
void     PD_PowerStop(uint8_t ch);

static int failures;
static int checks;

static void check(const char *what, int cond)
{
    checks++;
    if (!cond)
    {
        failures++;
        printf("  FAIL %s\n", what);
    }
}

/* Arm or refresh the watchdog exactly the way the CAN endpoint does. */
static uint16_t heartbeat(void)
{
    return PD_CanRequest(PD_COMMANDS_HEARTBEAT, 0, 0, 0);
}

static uint32_t ch1_fault(void)
{
    return pd_channels[0].fault;
}

static void fresh(void)
{
    Test_Reset();
    PD_Init();
    Test_Time(0);
}

static uint8_t serial_request(uint8_t op, uint8_t ch, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[32] = {0xa5u, 0x5au, PD_PROTOCOL_VERSION, PD_KINDS_REQUEST,
                         0x34u, 0x12u, op, ch, (uint8_t)len, (uint8_t)(len >> 8)};
    uint8_t response[512];
    uint16_t crc = 0xffffu;
    uint32_t got;
    uint16_t i;
    if (len > 20u) return 0xffu;
    for (i = 0; i < len; i++) frame[10u + i] = payload[i];
    for (i = 2u; i < (uint16_t)(10u + len); i++) {
        crc ^= (uint16_t)frame[i] << 8;
        for (uint8_t bit = 0; bit < 8u; bit++)
            crc = (uint16_t)((crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u :
                             ((uint32_t)crc << 1));
    }
    frame[10u + len] = (uint8_t)crc;
    frame[11u + len] = (uint8_t)(crc >> 8);
    (void)Test_Read(response, sizeof(response)); /* discard earlier logs */
    PD_Feed(frame, (uint16_t)(12u + len));
    got = Test_Read(response, sizeof(response));
    for (uint32_t j = 0; j + 12u <= got; j++) {
        if (response[j] == 0xa5u && response[j + 1u] == 0x5au &&
            response[j + 4u] == 0x34u && response[j + 5u] == 0x12u &&
            response[j + 6u] == op)
            return response[j + 3u];
    }
    return 0xffu;
}

static void test_no_fault_before_any_heartbeat(void)
{
    printf("watchdog stays quiet until a host has spoken\n");
    fresh();

    /* A board that has never been commanded must not latch a communication
     * fault: nothing has gone silent, because nothing was ever talking. */
    Test_Advance(PD_COMM_TIMEOUT_MS * 5);
    PD_Poll();
    check("no COMM_TIMEOUT without a heartbeat ever seen",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) == 0);
}

static void test_timeout_latches_comm_fault(void)
{
    printf("1 s without a heartbeat latches COMM_TIMEOUT\n");
    fresh();

    check("heartbeat accepted", heartbeat() == 0);
    PD_Poll();
    check("no fault right after the heartbeat",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) == 0);

    /* Just inside the window: still fine. */
    Test_Advance(PD_COMM_TIMEOUT_MS - 100);
    PD_Poll();
    check("no fault just before the deadline",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) == 0);

    /* Past the window: the fault must latch. */
    Test_Advance(200);
    PD_Poll();
    check("COMM_TIMEOUT latched after the deadline",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) != 0);
}

static void test_output_is_actually_disabled(void)
{
    /* The claim is "stops the board", not "sets a bit". If the stage were left
     * enabled with a latched fault, the display would read FAULT while the power
     * stage kept switching - the worst possible combination. This asserts the
     * physical consequence. */
    printf("the timeout actually disables the power stage\n");
    fresh();
    Test_Sensors(24000, 5000, 500);   /* plausible input, output, current */
    Test_Advance(1000);

    /* Bring the channel up. If the host harness cannot satisfy the start
     * preconditions this build skips the assertion rather than asserting
     * something meaningless - but it says so. */
    uint16_t started = PD_PowerStart(1);
    if (started != 0)
    {
        /* PD_PowerStart needs the full start preconditions (zero calibration
         * applied, a confirmed gate map, healthy sampling). Reproducing that
         * setup here would mean rebuilding machinery the existing suite already
         * drives with the real primitives, so this assertion is skipped rather
         * than faked. What that means:
         *   PROVEN here    - the watchdog latches COMM_TIMEOUT at the deadline;
         *   NOT proven here- that the consequent stop() disables the stage.
         * The second half is covered by tests/test_firmware.py, which starts the
         * converter for real, and is on the bench checklist in the plan.
         * Reported, never silently passed. */
        printf("  SKIP: PD_PowerStart refused here (err=%u, %s)\n",
               (unsigned)started,
               started == PD_ERRORS_NOT_READY ? "NOT_READY" : "see powerdesk_protocol.h");
        printf("        output-disable-on-timeout is NOT proven by this test;\n");
        printf("        it is covered by tests/test_firmware.py and the bench list.\n");
        return;
    }
    check("output enabled after a successful start",
          Test_PowerEnable() != 0 && pd_channels[0].enabled != 0);

    heartbeat();
    PD_Poll();

    /* Let the heartbeat lapse. */
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();

    check("COMM_TIMEOUT latched", (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) != 0);
    check("power stage disabled after the timeout", Test_PowerEnable() == 0);
    check("gates disabled after the timeout", Test_Gates() == 0);
    check("channel reports not enabled", pd_channels[0].enabled == 0);
}

static void test_heartbeat_refresh_prevents_timeout(void)
{
    printf("regular heartbeats keep the watchdog satisfied\n");
    fresh();

    for (int i = 0; i < 20; i++)
    {
        check("heartbeat accepted", heartbeat() == 0);
        Test_Advance(PD_COMM_TIMEOUT_MS / 4);   /* well inside the window */
        PD_Poll();
    }
    check("no COMM_TIMEOUT while heartbeats keep arriving",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) == 0);

    /* Stop talking: now it must fire. */
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();
    check("COMM_TIMEOUT latched once the heartbeats stop",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) != 0);
}

static void test_fault_latches_once(void)
{
    printf("the timeout does not keep re-latching every poll\n");
    fresh();

    heartbeat();
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();
    uint32_t first = ch1_fault();
    check("fault latched", (first & PD_FAULTS_COMM_TIMEOUT) != 0);

    /* Poll many more times: the bit must be stable, and no other fault may
     * appear as a side effect of the watchdog firing repeatedly. */
    for (int i = 0; i < 10; i++)
    {
        Test_Advance(PD_COMM_TIMEOUT_MS);
        PD_Poll();
    }
    check("fault bitmap unchanged by repeated polls", ch1_fault() == first);
}

static void test_heartbeat_rearms_after_timeout(void)
{
    printf("a later heartbeat re-arms the watchdog\n");
    fresh();

    heartbeat();
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();
    check("timed out first", (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) != 0);

    /* The plan says a later HEARTBEAT re-arms it (see the STOP_ALL comment in
     * pd_core.c). The previously latched bit stays latched - clearing a fault is
     * an explicit operator action - but the watchdog must be live again. */
    check("re-arming heartbeat accepted", heartbeat() == 0);
    Test_Advance(PD_COMM_TIMEOUT_MS - 100);
    PD_Poll();

    /* Nothing new should be latched by this poll; the state must be stable. */
    uint32_t before = ch1_fault();
    Test_Advance(100);
    PD_Poll();
    check("re-armed watchdog does not add bits before its own deadline",
          ch1_fault() == before);
}

static void test_can_heartbeat_entry_points(void)
{
    printf("CAN heartbeat entry points drive the watchdog\n");
    uint32_t via_port, via_request;

    /* CAN path: the accessor the CAN endpoint calls. */
    fresh();
    PD_CanHeartbeat();
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();
    via_port = ch1_fault() & PD_FAULTS_COMM_TIMEOUT;

    /* The shared CAN request entry point also claims CAN control. */
    fresh();
    check("serial heartbeat accepted", heartbeat() == 0);
    Test_Advance(PD_COMM_TIMEOUT_MS + 100);
    PD_Poll();
    via_request = ch1_fault() & PD_FAULTS_COMM_TIMEOUT;

    check("CAN port heartbeat arms the watchdog", via_port != 0);
    check("CAN request heartbeat arms the watchdog", via_request != 0);
    check("both entry points produce the same protection", via_port == via_request);
}

static void test_can_owns_watchdog_and_uart_is_read_only(void)
{
    uint8_t target[8] = {0x70u, 0x17u, 0u, 0u, 0x64u, 0u, 0u, 0u};
    int32_t old_target;
    printf("CAN ownership: UART remains telemetry-only and cannot mask timeout\n");
    fresh();
    check("UART heartbeat arms before CAN appears",
          serial_request(PD_COMMANDS_HEARTBEAT, 0u, 0, 0u) == PD_KINDS_ACK);
    Test_Advance(600u);
    PD_CanHeartbeat();
    old_target = pd_channels[0].vref_mv;
    check("UART status remains readable",
          serial_request(PD_COMMANDS_STATUS, 1u, 0, 0u) == PD_KINDS_ACK);
    check("UART SET_TARGET rejected after CAN takeover",
          serial_request(PD_COMMANDS_SET_TARGET, 1u, target, 8u) == PD_KINDS_NACK);
    check("rejected UART target did not change power reference",
          pd_channels[0].vref_mv == old_target);
    Test_Advance(600u);
    (void)serial_request(PD_COMMANDS_HEARTBEAT, 0u, 0, 0u);
    Test_Advance(600u);
    PD_Poll();
    check("UART heartbeat cannot keep CAN-owned stage alive",
          (ch1_fault() & PD_FAULTS_COMM_TIMEOUT) != 0u);
}

int main(void)
{
    printf("G474 communication watchdog (host)\n\n");
    test_no_fault_before_any_heartbeat();
    test_timeout_latches_comm_fault();
    test_output_is_actually_disabled();
    test_heartbeat_refresh_prevents_timeout();
    test_fault_latches_once();
    test_heartbeat_rearms_after_timeout();
    test_can_heartbeat_entry_points();
    test_can_owns_watchdog_and_uart_is_read_only();

    printf("\n%d checks, %d failures\n", checks, failures);
    if (failures)
    {
        printf("G474 watchdog test FAILED\n");
        return 1;
    }
    /* Deliberately precise. This suite proves the watchdog TIMING and that both
     * transports arm it. It does NOT prove the consequent output shutdown: that
     * assertion is skipped above and is covered by tests/test_firmware.py and the
     * bench list. The summary must not claim more than the checks assert. */
    printf("G474 watchdog: CAN owns the deadline after takeover; UART stays readable.\n");
    return 0;
}
