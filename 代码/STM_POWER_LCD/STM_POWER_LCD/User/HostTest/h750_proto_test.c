/* Host test for the H750 CAN protocol state machine (User/App/Src/can_proto.c).
 *
 * WHAT THIS EXISTS TO PROVE. The plan states two safety properties in words:
 *
 *   "H750 updates the 'output ON' display only from the G474 status report."
 *   "On link loss show 'device offline', and do NOT resend the old start command."
 *
 * Both are properties of the state machine, not of the hardware, so they can be
 * checked on a PC. Until now they were only verified by reading the code - grep
 * can show that output_on is written in the right places, but not that the
 * timing and ordering actually behave.
 *
 * HOW. can_proto.c is already FreeRTOS-free. It needs only:
 *     canif_init/send/receive   - faked here, the test owns the bus
 *     ui_model()                - the real model, compiled in
 *     HAL_GetTick()             - the test owns the clock
 *     LOGI/LOGW/LOGE            - shimmed in User/HostTest/shim
 * The shim directory goes first on the include path, so the real bsp_log.h and
 * main.h (which drag in the HAL) are shadowed. can_proto.c is unmodified.
 *
 * BUILD/RUN: tools/run_h750_proto_test.py
 */
#include "can_proto.h"
#include "can_protocol.h"
#include "ui_model.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Fakes: the clock and the bus
 * ------------------------------------------------------------------------- */
static uint32_t g_now_ms;

uint32_t HAL_GetTick(void)
{
    return g_now_ms;
}

static void tick(uint32_t ms)
{
    g_now_ms += ms;
}

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} tx_frame;

#define TX_CAP 64
static tx_frame g_tx[TX_CAP];
static int      g_tx_n;

/* Frames the test wants can_proto to receive, delivered on the next poll. */
#define RX_CAP 16
static tx_frame g_rx[RX_CAP];
static int      g_rx_n;
static int      g_rx_head;

int canif_init(void)
{
    return 0;
}

int canif_send(uint32_t id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (g_tx_n < TX_CAP)
    {
        g_tx[g_tx_n].id = id;
        g_tx[g_tx_n].dlc = dlc;
        memset(g_tx[g_tx_n].data, 0, sizeof(g_tx[g_tx_n].data));
        if (data && dlc)
            memcpy(g_tx[g_tx_n].data, data, dlc);
        g_tx_n++;
    }
    return 0;
}

int canif_receive(void *out, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (g_rx_head >= g_rx_n)
        return 0;
    /* can_proto passes a canif_frame_t; keep the layout in step with can_if.h */
    {
        struct frame { uint32_t id; uint8_t dlc; uint8_t data[8]; } *f = out;
        f->id = g_rx[g_rx_head].id;
        f->dlc = g_rx[g_rx_head].dlc;
        memcpy(f->data, g_rx[g_rx_head].data, sizeof(f->data));
    }
    g_rx_head++;
    return 1;
}

void canif_rx_from_isr(void) { }

/* --- logging shim --------------------------------------------------------- */
#define LOG_LINES 64
static char g_log[LOG_LINES][48];
static int  g_log_n;

void host_log_reset(void) { g_log_n = 0; }

void host_log_probe(const char *level, const char *tag, const char *fmt, ...)
{
    /* Consume the format and its arguments so nothing is "unused", and render a
     * real line: the test asserts on the text the firmware would actually print. */
    char body[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    if (g_log_n < LOG_LINES)
    {
        snprintf(g_log[g_log_n], sizeof(g_log[0]), "%s %s %s", level, tag ? tag : "", body);
        g_log_n++;
    }
}

int host_log_count(const char *needle)
{
    int n = 0, i;
    for (i = 0; i < g_log_n; i++)
        if (strstr(g_log[i], needle))
            n++;
    return n;
}

/* ---------------------------------------------------------------------------
 * Test helpers
 * ------------------------------------------------------------------------- */
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

static void tx_reset(void) { g_tx_n = 0; }

static const tx_frame *tx_find(uint32_t id)
{
    int i;
    for (i = 0; i < g_tx_n; i++)
        if (g_tx[i].id == id)
            return &g_tx[i];
    return 0;
}

static int tx_count(uint32_t id)
{
    int i, n = 0;
    for (i = 0; i < g_tx_n; i++)
        if (g_tx[i].id == id)
            n++;
    return n;
}

static void rx_reset(void) { g_rx_n = 0; g_rx_head = 0; }

static void rx_push(uint32_t id, const uint8_t *d, uint8_t dlc)
{
    if (g_rx_n >= RX_CAP)
        return;
    g_rx[g_rx_n].id = id;
    g_rx[g_rx_n].dlc = dlc;
    memset(g_rx[g_rx_n].data, 0, 8);
    if (d && dlc)
        memcpy(g_rx[g_rx_n].data, d, dlc);
    g_rx_n++;
}

/* Build the frames the G474 would send. Offsets are the shared contract. */
static void g474_measured(uint16_t v_mv, uint16_t i_ma, uint8_t state, uint8_t fault_lo)
{
    uint8_t d[8];
    d[0] = (uint8_t)(v_mv & 0xFF);
    d[1] = (uint8_t)(v_mv >> 8);
    d[2] = (uint8_t)(i_ma & 0xFF);
    d[3] = (uint8_t)(i_ma >> 8);
    d[4] = CANP_CH1;
    d[5] = state;
    d[6] = fault_lo;
    d[7] = 0;
    rx_push(CANP_ID_MEASURED, d, 8);
}

static void g474_fault(uint32_t bits)
{
    uint8_t d[8];
    d[0] = CANP_CH1;
    d[1] = 0;
    d[2] = (uint8_t)(bits & 0xFF);
    d[3] = (uint8_t)((bits >> 8) & 0xFF);
    d[4] = (uint8_t)((bits >> 16) & 0xFF);
    d[5] = (uint8_t)((bits >> 24) & 0xFF);
    d[6] = 0;
    d[7] = 0;
    rx_push(CANP_ID_FAULT, d, 8);
}

static void g474_ack(uint16_t seq, uint8_t result, uint8_t err, uint16_t applied_v)
{
    uint8_t d[8];
    d[0] = (uint8_t)(seq & 0xFF);
    d[1] = (uint8_t)(seq >> 8);
    d[2] = CANP_CH1;
    d[3] = result;
    d[4] = err;
    d[5] = (uint8_t)(applied_v & 0xFF);
    d[6] = (uint8_t)(applied_v >> 8);
    d[7] = CANP_STATE_OFF;
    rx_push(CANP_ID_SET_ACK, d, 8);
}

static void g474_capability(uint16_t vmin, uint16_t vmax, uint16_t imax)
{
    uint8_t d[8];
    d[0] = CANP_CH1;
    d[1] = (uint8_t)(vmin & 0xFF);
    d[2] = (uint8_t)(vmin >> 8);
    d[3] = (uint8_t)(vmax & 0xFF);
    d[4] = (uint8_t)(vmax >> 8);
    d[5] = (uint8_t)(imax & 0xFF);
    d[6] = (uint8_t)(imax >> 8);
    d[7] = CANP_CAP_CH_PRESENT | CANP_CAP_CV_CC;
    rx_push(CANP_ID_CAPABILITY, d, 8);
}

static void g474_ready(void)
{
    const tx_frame *req;
    tx_reset();
    rx_reset();
    g474_measured(0, 0, CANP_STATE_OFF, 0);
    can_proto_poll(1);
    req = tx_find(CANP_ID_SET); /* automatic capability query on link-up */
    if (req != NULL) {
        uint16_t seq = (uint16_t)(req->data[CANP_SET_SEQ] |
                                  (req->data[CANP_SET_SEQ + 1] << 8));
        rx_reset();
        g474_ack(seq, CANP_RES_ACCEPTED, CANP_ERR_NONE, 0);
        can_proto_poll(1);
    }
    tx_reset();
    rx_reset();
}

/* ---------------------------------------------------------------------------
 * Tests
 * ------------------------------------------------------------------------- */
static void test_boot_stays_offline(void)
{
    ui_model_t *m;
    printf("boot: no power board yet\n");
    g_now_ms = 1000;
    ui_model_init();
    can_proto_init();

    m = ui_model();
    check("production source is CAN, never MOCK", m->source == UI_SRC_CAN);
    check("link is not up", m->link_up == 0);
    check("output is off", m->ch1.output_on == 0);
    can_proto_poll(0);
    check("first poll without telemetry stays offline", m->link_up == 0);
    tick(20);
    can_proto_poll(0);
    check("startup grace period cannot invent a link", m->link_up == 0);

    /* No board means OFFLINE and no simulated output. */
    tick(2000);
    can_proto_poll(1);
    check("still CAN source after 2 s of silence", ui_model()->source == UI_SRC_CAN);
    check("OFFLINE shown", ui_model()->link_up == 0);
}

static void test_output_on_requires_telemetry(void)
{
    printf("safety: 'output ON' only from reported telemetry\n");
    g_now_ms = 10000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();
    g474_ready();

    /* The operator presses START. That only SENDS a request. */
    can_proto_request_start();
    check("a START request was transmitted", tx_find(CANP_ID_SET) != 0);
    check("output is NOT shown on yet", ui_model()->ch1.output_on == 0);

    /* An ACK alone must still not light the display. */
    {
        const tx_frame *req = tx_find(CANP_ID_SET);
        uint16_t seq = (uint16_t)(req->data[CANP_SET_SEQ] | (req->data[CANP_SET_SEQ + 1] << 8));
        g474_ack(seq, CANP_RES_ACCEPTED, CANP_ERR_NONE, 0);
    }
    can_proto_poll(1);
    check("output still NOT shown after the ACK alone", ui_model()->ch1.output_on == 0);

    /* Now the status report arrives with the board running in CV. */
    rx_reset();
    g474_measured(5000, 100, CANP_STATE_CV, 0);
    can_proto_poll(1);
    check("output shown once telemetry reports CV", ui_model()->ch1.output_on == 1);
    check("CV mode shown", ui_model()->ch1.cv_mode == 1);
    check("measured voltage taken from telemetry", ui_model()->ch1.v_out_mv == 5000);

    /* CC must be recognised too, and OFF must clear the display. */
    rx_reset();
    g474_measured(4800, 1000, CANP_STATE_CC, 0);
    can_proto_poll(1);
    check("CC mode shown", ui_model()->ch1.cv_mode == 0);

    rx_reset();
    g474_measured(0, 0, CANP_STATE_OFF, 0);
    can_proto_poll(1);
    check("OFF clears the output display", ui_model()->ch1.output_on == 0);
}

static void test_fault_forces_output_off(void)
{
    printf("safety: a fault report turns the display off\n");
    g_now_ms = 20000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();

    g474_measured(5000, 100, CANP_STATE_CV, 0);
    can_proto_poll(1);
    check("running first", ui_model()->ch1.output_on == 1);

    rx_reset();
    g474_fault(CANP_FAULT_OTP);
    can_proto_poll(1);
    check("fault latched", ui_model()->ch1.fault_latched == 1);
    check("fault bits carried", ui_model()->ch1.fault_bits == CANP_FAULT_OTP);
    check("output shown off after the fault", ui_model()->ch1.output_on == 0);
}

static void test_link_loss_clears_without_resending(void)
{
    printf("safety: link loss shows offline and does NOT resend START\n");
    g_now_ms = 30000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();

    /* Get to a running state. */
    g474_measured(5000, 100, CANP_STATE_CV, 0);
    can_proto_poll(1);
    check("running", ui_model()->ch1.output_on == 1);
    check("link up", ui_model()->link_up == 1);

    /* Silence for longer than CANP_LINK_LOST_MS. */
    tick(CANP_LINK_LOST_MS + 100);
    tx_reset();
    can_proto_poll(1);

    check("link reported down", ui_model()->link_up == 0);
    check("output display cleared on link loss", ui_model()->ch1.output_on == 0);
    check("stale measurements cleared on link loss", ui_model()->ch1.v_out_mv == 0 && ui_model()->ch1.i_out_ma == 0);
    /* The plan is explicit about this one. */
    check("no START was resent", tx_count(CANP_ID_SET) == 0);

    /* Keep polling: it must not retry the old command either. */
    tick(5000);
    can_proto_poll(1);
    check("still no START resent after 5 s", tx_count(CANP_ID_SET) == 0);
}

static void test_capability_sets_the_limits(void)
{
    printf("the UI takes its range from the capability report\n");
    g_now_ms = 40000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();

    /* Move the setpoint outside the range the board is about to declare. */
    ui_model()->ch1.v_set_mv = 30000;
    ui_model()->ch1.i_limit_ma = 4000;

    g474_capability(2000, 18000, 5000);
    can_proto_poll(1);

    check("vmax taken from the board", ui_model()->ch1.v_max_mv == 18000);
    check("vmin taken from the board", ui_model()->ch1.v_min_mv == 2000);
    check("imax taken from the board", ui_model()->ch1.i_max_ma == 5000);
    /* 30000 was outside the new window, so it must have been pulled in. */
    check("setpoint clamped into the reported window", ui_model()->ch1.v_set_mv == 18000);
    check("current limit left alone when inside the window", ui_model()->ch1.i_limit_ma == 4000);
}

static void test_command_sequence_and_retry(void)
{
    printf("commands carry a sequence and are retried a bounded number of times\n");
    g_now_ms = 50000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();
    g474_ready();

    can_proto_request_set(7000, 500);

    /* No answer: the protocol retries, but must give up. */
    check("one SET frame sent", tx_count(CANP_ID_SET) == 1);
    check("command marked pending right after sending", can_proto_command_pending() == 1);

    /* No answer: the state machine retries, but it must give up rather than
     * flood the bus for ever. Sending once plus two retries is the design. */
    tick(400); g474_measured(0, 0, CANP_STATE_OFF, 0); can_proto_poll(1);
    check("retried once while unanswered", tx_count(CANP_ID_SET) >= 2);
    check("still pending while awaiting the answer", can_proto_command_pending() == 1);

    tick(400); g474_measured(0, 0, CANP_STATE_OFF, 0); can_proto_poll(1);
    tick(400); g474_measured(0, 0, CANP_STATE_OFF, 0); can_proto_poll(1);
    check("gave up instead of flooding", tx_count(CANP_ID_SET) <= 4);
    check("pending cleared after giving up", can_proto_command_pending() == 0);

    /* After giving up it must stay quiet - no further attempts. */
    {
        int before = tx_count(CANP_ID_SET);
        tick(3000); can_proto_poll(1);
        check("no retries after giving up", tx_count(CANP_ID_SET) == before);
    }

    /* Now a fresh command, answered promptly: the ACK must clear pending. */
    g474_ready();
    can_proto_request_set(7000, 500);
    check("new command is pending", can_proto_command_pending() == 1);
    {
        const tx_frame *req = tx_find(CANP_ID_SET);
        uint16_t seq = (uint16_t)(req->data[CANP_SET_SEQ] | (req->data[CANP_SET_SEQ + 1] << 8));
        rx_reset();
        g474_ack(seq, CANP_RES_ACCEPTED, CANP_ERR_NONE, 7000);
        can_proto_poll(1);
    }
    check("pending cleared by the ACK", can_proto_command_pending() == 0);

    tx_reset();
    tick(2000);
    can_proto_poll(1);
    check("no further retries after the ACK", tx_count(CANP_ID_SET) == 0);
}

static void test_stale_ack_is_ignored(void)
{
    printf("an ACK for an old sequence must not be applied\n");
    g_now_ms = 60000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();
    g474_ready();

    /* Note: can_proto_request_set() only QUEUES a command, it does not move the
     * model's setpoint - that is the UI's job on confirmation. So the thing to
     * assert is that the stale ACK does not move it either. */
    {
        uint16_t before = ui_model()->ch1.v_set_mv;
        can_proto_request_set(7000, 500);
        /* An acknowledgement for a sequence we never sent, delivered immediately
         * so the command is certainly still outstanding. */
        rx_reset();
        g474_ack(4242, CANP_RES_ACCEPTED, CANP_ERR_NONE, 12345);
        can_proto_poll(1);

        check("stale ACK did not move the setpoint", ui_model()->ch1.v_set_mv == before);
        check("command still pending after a stale ACK", can_proto_command_pending() == 1);
    }
}

static void test_heartbeat_cadence(void)
{
    printf("heartbeat runs at the agreed period\n");
    g_now_ms = 70000;
    ui_model_init();
    can_proto_init();
    tx_reset();
    rx_reset();

    can_proto_poll(1);
    can_proto_poll(1);
    check("first poll emits a heartbeat", tx_count(CANP_ID_HEARTBEAT) == 1);

    tick(100);
    can_proto_poll(1);
    check("not yet due after 100 ms", tx_count(CANP_ID_HEARTBEAT) == 1);

    tick(CANP_HEARTBEAT_PERIOD_MS);
    can_proto_poll(1);
    check("due after the period", tx_count(CANP_ID_HEARTBEAT) == 2);

    /* The plan requires the G474 to stop without a heartbeat, so the H750 must
     * keep sending even while nothing is connected. */
    tx_reset();
    tick(CANP_HEARTBEAT_PERIOD_MS * 3);
    can_proto_poll(1);
    check("keeps beating with no board attached", tx_count(CANP_ID_HEARTBEAT) >= 1);
}

static void test_offline_and_fault_guards(void)
{
    printf("offline START is blocked; high fault bits survive telemetry\n");
    g_now_ms = 80000;
    ui_model_init();
    can_proto_init();
    tx_reset(); rx_reset();
    can_proto_request_start();
    check("offline START sends no command", tx_count(CANP_ID_SET) == 0);
    g474_ready();
    g474_fault(CANP_FAULT_SENSOR);
    can_proto_poll(1);
    check("sensor fault displayed", ui_model()->ch1.fault_latched != 0);
    g474_measured(0, 0, CANP_STATE_FAULT, 0);
    can_proto_poll(1);
    check("low-byte telemetry does not clear sensor fault",
          ui_model()->ch1.fault_bits == CANP_FAULT_SENSOR);
    tx_reset();
    can_proto_request_start();
    check("faulted START sends no command", tx_count(CANP_ID_SET) == 0);
}

static void test_set_must_ack_before_start(void)
{
    const tx_frame *req;
    uint16_t seq;
    printf("SET must be accepted before a queued START is sent\n");
    g_now_ms = 90000;
    ui_model_init(); can_proto_init(); tx_reset(); rx_reset(); g474_ready();
    can_proto_request_set(7000, 500);
    can_proto_request_start();
    check("only SET sent while awaiting its ACK", tx_count(CANP_ID_SET) == 1);
    req = tx_find(CANP_ID_SET);
    seq = (uint16_t)(req->data[0] | (req->data[1] << 8));
    g474_ack(seq, CANP_RES_ACCEPTED, CANP_ERR_NONE, 7000);
    can_proto_poll(1);
    check("START sent only after accepted SET", tx_count(CANP_ID_SET) == 2);
    check("queued command is START", g_tx[g_tx_n - 1].data[CANP_SET_OP] == CANP_OP_START);

    ui_model_init(); can_proto_init(); tx_reset(); rx_reset(); g474_ready();
    can_proto_request_set(7000, 500);
    can_proto_request_start();
    req = tx_find(CANP_ID_SET);
    seq = (uint16_t)(req->data[0] | (req->data[1] << 8));
    g474_ack(seq, CANP_RES_REJECTED, CANP_ERR_RANGE_I, 5000);
    can_proto_poll(1);
    check("rejected SET discards queued START", tx_count(CANP_ID_SET) == 1);
}

int main(void)
{
    printf("H750 CAN protocol state machine (host)\n\n");
    test_boot_stays_offline();
    test_output_on_requires_telemetry();
    test_fault_forces_output_off();
    test_link_loss_clears_without_resending();
    test_capability_sets_the_limits();
    test_command_sequence_and_retry();
    test_stale_ack_is_ignored();
    test_heartbeat_cadence();
    test_offline_and_fault_guards();
    test_set_must_ack_before_start();

    printf("startup readiness and active transitions\n");
    g_now_ms = 120000;
    ui_model_init(); can_proto_init(); tx_reset(); rx_reset(); g474_ready();
    g474_capability(2000, 44000, 5000);
    g_rx[g_rx_n - 1].data[7] |= CANP_CAP_READY_KNOWN;
    can_proto_poll(0);
    check("readiness feature is recognised", ui_model()->ch1.ready_known && !ui_model()->ch1.ready);
    tx_reset(); can_proto_request_start();
    check("START blocked until board reports ready", tx_count(CANP_ID_SET) == 0);
    rx_reset(); g474_capability(2000, 44000, 5000);
    g_rx[g_rx_n - 1].data[7] |= CANP_CAP_READY_KNOWN | CANP_CAP_READY;
    can_proto_poll(0); can_proto_request_start();
    check("START permitted after readiness report", tx_count(CANP_ID_SET) == 1);
    rx_reset(); g474_measured(100, 0, CANP_STATE_STARTING, 0); can_proto_poll(0);
    check("reported soft start is an active output", ui_model()->ch1.output_on && ui_model()->ch1.run_state == CANP_STATE_STARTING);
    can_proto_request_stop();
    check("STOP available during soft start", g_tx[g_tx_n - 1].data[CANP_SET_OP] == CANP_OP_STOP);
    rx_reset(); g474_measured(50, 0, CANP_STATE_STOPPING, 0); can_proto_poll(0);
    check("reported stopping stays active until OFF", ui_model()->ch1.output_on && ui_model()->ch1.run_state == CANP_STATE_STOPPING);
    rx_reset(); g474_measured(0, 0, CANP_STATE_OFF, 0); can_proto_poll(0);
    check("reported OFF completes the stop", !ui_model()->ch1.output_on);
    rx_reset(); g474_capability(2000, 44000, 5000);
    g_rx[g_rx_n - 1].data[7] |= CANP_CAP_READY_KNOWN | CANP_CAP_INIT_FAILED;
    can_proto_poll(0);
    check("startup failure is reported separately", ui_model()->ch1.startup_failed && !ui_model()->ch1.ready);
    tx_reset(); can_proto_request_start();
    check("failed calibration keeps START blocked", tx_count(CANP_ID_SET) == 0);

    {
        tx_frame tf = {0};
        tf.id = CANP_ID_TEMPERATURE; tf.dlc = 8;
        canp_put_u16(tf.data, 2500);
        canp_put_u16(tf.data + 2, (uint16_t)-500);
        tf.data[4] = 1; tf.data[5] = 3;
        g_rx[0] = tf; g_rx_n = 1; g_rx_head = 0;
        can_proto_poll(0);
        check("temperature receives signed values", ui_model()->ch1.tmos_cdeg == 2500 &&
              ui_model()->ch1.tl_cdeg == -500 && ui_model()->ch1.temp_valid == 3);
        uint32_t temp_at = ui_model()->ch1.temp_rx_ms;
        tf.data[4] = 2; tick(1); g_rx[0] = tf; g_rx_n = 1; g_rx_head = 0; can_proto_poll(0);
        check("CH2 temperature ignored", ui_model()->ch1.temp_rx_ms == temp_at);
        tf.data[4] = 1; tf.dlc = 7; g_rx[0] = tf; g_rx_n = 1; g_rx_head = 0; can_proto_poll(0);
        check("truncated temperature ignored", ui_model()->ch1.temp_rx_ms == temp_at);
        tf.dlc = 8; canp_put_u16(tf.data + 2, 0x8000); tf.data[5] = 3;
        g_rx[0] = tf; g_rx_n = 1; g_rx_head = 0; can_proto_poll(0);
        check("invalid sentinel cannot claim healthy coil", ui_model()->ch1.temp_valid == 1);
        tick(CANP_LINK_LOST_MS + 1);
        canp_put_u16(tf.data + 2, 4000); tf.data[5] = 3;
        g_rx[0] = tf; g_rx_n = 1; g_rx_head = 0; can_proto_poll(0);
        check("temperature alone cannot keep output link alive", !ui_model()->link_up);
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    if (failures)
    {
        printf("H750 protocol test FAILED\n");
        return 1;
    }
    printf("H750 protocol: the two safety properties hold\n");
    return 0;
}
