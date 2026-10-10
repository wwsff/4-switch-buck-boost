/* FDCAN2 glue for the power board CAN transport.
 *
 * DELIBERATE DESIGN: receive is POLLED from the same task that calls
 * PD_BoardPoll(), not queued from an interrupt.
 *
 * pd_core.c already has this shape on the UART side - the ISR only feeds bytes
 * and all work happens in task context - and both transports end up calling the
 * same dispatch() and touching the same pd_channels/fault state. Polling keeps
 * that single-threaded instead of adding a second writer. At the 1 ms task
 * period a command is answered within a millisecond, which is far inside what a
 * 250 ms heartbeat and an occasional setpoint need.
 *
 * This file is the ONLY place in the CAN path that touches the HAL, so
 * App/pd_can.c stays host-compilable. It is not covered by the host gate.
 */
#include "pd_can_port.h"

#include "pd_can.h"
#include "pd_core.h"
#include "pd_config.h"
#include "pd_power.h"
#include "pd_zero.h"
#include "main.h"

#include "fdcan.h"
#include <string.h>

/* Cadence ------------------------------------------------------------------ */
#define PD_CAN_TELEMETRY_MS 100u /* the HMI expects about this */
#define PD_CAN_FAULT_MS 200u     /* re-send a standing fault at this rate */
#define PD_CAN_QUEUE_ELMTS 4u
#define PD_CAN_DLC 8u
#define PD_CAN_TX_PENDING 8u

/* Identifiers, mirrored from pd_can.c (kept local: the codec owns decoding, this
   file only needs to know what to transmit). */
#define PD_CAN_ID_HEARTBEAT 0x100u
#define PD_CAN_ID_RESULT 0x190u
#define PD_CAN_ID_MEASURED 0x200u
#define PD_CAN_ID_FAULT 0x201u
#define PD_CAN_ID_CAPABILITY 0x210u

static uint8_t s_ready;
static uint8_t s_status_due, s_last_ready;
static uint32_t s_last_capability;
static uint8_t s_meas_seq;
static uint32_t s_last_telemetry;
static uint32_t s_last_fault;
static uint32_t s_last_fault_bits;
static uint16_t s_last_status; /* bit0 = have sent a status already */
static struct { uint32_t id; uint8_t data[PD_CAN_DLC]; } s_tx_pending[PD_CAN_TX_PENDING];
static uint8_t s_tx_count;

/* --------------------------------------------------------------------------- */
static int tx_hw(uint32_t id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef hdr;
    hdr.Identifier = id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = PD_CAN_DLC;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF; /* classic CAN */
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0u)
        return -1; /* bus busy: drop rather than block the 1 kHz task */
    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &hdr, (uint8_t *)data) == HAL_OK) ? 0 : -1;
}

static void flush_tx(void)
{
    while (s_tx_count != 0u && tx_hw(s_tx_pending[0].id, s_tx_pending[0].data) == 0)
    {
        s_tx_count--;
        if (s_tx_count != 0u)
            memmove(&s_tx_pending[0], &s_tx_pending[1],
                    (size_t)s_tx_count * sizeof(s_tx_pending[0]));
    }
}

static int tx(uint32_t id, const uint8_t *data)
{
    flush_tx();
    if (s_tx_count == 0u && tx_hw(id, data) == 0)
        return 0;
    /* Periodic measurements become stale; send a fresh one next period. */
    if (id == PD_CAN_ID_MEASURED || id == PD_CAN_ID_TEMPERATURE)
        return -1;
    if (s_tx_count >= PD_CAN_TX_PENDING)
        return -1;
    s_tx_pending[s_tx_count].id = id;
    memcpy(s_tx_pending[s_tx_count].data, data, PD_CAN_DLC);
    s_tx_count++;
    return 0;
}

/* --------------------------------------------------------------------------- */
int PD_CanPortInit(void)
{
    FDCAN_FilterTypeDef f;

    /* Accept every standard identifier into FIFO0; the codec decides what is
       meaningful. The HMI is the only talker on this bus. */
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0x000u;
    f.FilterID2 = 0x000u; /* mask 0 = accept all */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &f) != HAL_OK)
        return -1;

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return -1;

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
        return -1;

    s_ready = 1;
    s_tx_count = 0u;
    s_status_due = 1u;
    s_last_ready = 0xFFu;
    return 0;
}

/* --------------------------------------------------------------------------- */
static int send_capability(uint8_t ch)
{
    uint8_t d[PD_CAN_DLC];
    const PD_Channel *c = &pd_channels[ch - 1u];
    /* Advertise the CONFIGURED window for CH1 and nothing for CH2, exactly as
       the CAPS command does over the serial link, so the HMI's limits agree
       with what SET_TARGET will actually accept. */
    uint8_t present = (ch == 1u) ? 1u : 0u;
    (void)pd_can_encode_capability(ch, present ? (uint16_t)PD_BUCK_VMIN_MV : 0u,
                                   present ? (uint16_t)PD_BUCK_VMAX_MV : 0u,
                                   present ? (uint16_t)PD_IMAX_MA : 0u, present ? 1u : 0u, d);
    (void)c;
    d[7] |= PD_CAN_CAP_READY_KNOWN;
    if (present && PD_PowerReady(ch)) d[7] |= PD_CAN_CAP_READY;
    if (present && PD_ZeroStartupState() == PD_ZERO_BOOT_FAILED) d[7] |= PD_CAN_CAP_INIT_FAILED;
    return tx(PD_CAN_ID_CAPABILITY, d);
}

static void send_result(uint16_t seq, uint8_t ch, uint16_t pd_error)
{
    uint8_t d[PD_CAN_DLC];
    uint8_t state = 0u;
    uint16_t applied = 0u;
    if (ch >= 1u && ch <= 2u)
    {
        const PD_Channel *c = &pd_channels[ch - 1u];
        state = pd_can_map_state(c->state, c->enabled);
        /* Echo what the board ACCEPTED, not what was asked for; the HMI shows
           this value and must not display a target the board refused. */
        applied = (uint16_t)((c->vref_mv > 0) ? c->vref_mv : 0);
    }
    (void)pd_can_encode_result(seq, ch, pd_error, applied, state, d);
    (void)tx(PD_CAN_ID_RESULT, d);
}

static void handle_command(const pd_can_request *r)
{
    uint16_t err;
    uint8_t payload[4];

    /* QUERY_CAP needs no dispatcher call: answer from the same source CAPS uses. */
    if (r->can_op == 0x05u) /* PD_CAN_OP_QUERY_CAP */
    {
        send_capability(r->channel);
        send_result(r->seq, r->channel, 0u);
        return;
    }

    if (r->can_op == 0x03u) /* PD_CAN_OP_SET_SETPOINT */
    {
        /* The serial protocol carries millivolt/milliamp as int32; the CAN frame
           has 16 bits each. Widened here, validated by the shared dispatcher. */
        int32_t v = (int32_t)r->v_mv;
        int32_t i = (int32_t)r->i_ma;
        payload[0] = (uint8_t)(v & 0xff);
        payload[1] = (uint8_t)((v >> 8) & 0xff);
        payload[2] = (uint8_t)((v >> 16) & 0xff);
        payload[3] = (uint8_t)((v >> 24) & 0xff);
        {
            uint8_t body[8];
            body[0] = payload[0];
            body[1] = payload[1];
            body[2] = payload[2];
            body[3] = payload[3];
            body[4] = (uint8_t)(i & 0xff);
            body[5] = (uint8_t)((i >> 8) & 0xff);
            body[6] = (uint8_t)((i >> 16) & 0xff);
            body[7] = (uint8_t)((i >> 24) & 0xff);
            err = PD_CanRequest(PD_COMMANDS_SET_TARGET, r->channel, body, 8u);
        }
    }
    else if (r->can_op == 0x01u || r->can_op == 0x02u) /* START / STOP */
    {
        payload[0] = (r->can_op == 0x01u) ? 1u : 0u;
        err = PD_CanRequest(PD_COMMANDS_OUTPUT, r->channel, payload, 1u);
    }
    else if (r->can_op == 0x04u) /* CLEAR_FAULT */
    {
        err = PD_CanRequest(PD_COMMANDS_CLEAR_FAULT, r->channel, payload, 0u);
    }
    else
    {
        err = PD_ERRORS_UNSUPPORTED;
    }

    send_result(r->seq, r->channel, err);
    s_status_due = 1u; /* report real start/stop transition without waiting 100ms */
}

static void drain_rx(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0u)
    {
        FDCAN_RxHeaderTypeDef hdr;
        uint8_t data[PD_CAN_DLC];
        pd_can_request req;

        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
            return;
        if (hdr.IdType != FDCAN_STANDARD_ID)
            continue;
        if (!pd_can_decode(hdr.Identifier, data, (uint8_t)hdr.DataLength, &req))
            continue;

        if (req.kind == PD_CAN_KIND_HEARTBEAT)
        {
            /* Arms or refreshes the SHARED watchdog. Any recognised master
               speaking keeps the stage alive; silence from all of them for
               PD_COMM_TIMEOUT_MS stops it (see PD_CanRequest's note). */
            PD_CanHeartbeat();
        }
        else if (req.kind == PD_CAN_KIND_COMMAND)
        {
            handle_command(&req);
        }
    }
}

/* --------------------------------------------------------------------------- */
void PD_CanPortPoll(void)
{
    uint32_t now;

    if (!s_ready)
        return;

    flush_tx();
    drain_rx();
    flush_tx();

    now = HAL_GetTick();

    {
        uint8_t ready = PD_PowerReady(1u);
        if (ready != s_last_ready || (uint32_t)(now - s_last_capability) >= 500u)
        {
            if (send_capability(1u) == 0) {
                s_last_ready = ready;
                s_last_capability = now;
            }
        }
    }
    if (s_status_due || (uint32_t)(now - s_last_telemetry) >= PD_CAN_TELEMETRY_MS)
    {
        uint8_t d[PD_CAN_DLC];
        uint8_t ch = 1u; /* this HMI serves one output */
        const PD_Channel *c = &pd_channels[ch - 1u];

        s_last_telemetry = now;
        s_status_due = 0u;

        (void)pd_can_encode_measured((uint16_t)((c->vout_mv > 0) ? c->vout_mv : 0),
                                     (uint16_t)((c->iout_ma > 0) ? c->iout_ma : 0), ch,
                                     pd_can_map_state(c->state, c->enabled),
                                     pd_can_map_fault(c->fault), s_meas_seq++, d);
        (void)tx(PD_CAN_ID_MEASURED, d);
        uint8_t temp_valid = 0u;
        if (c->valid & PD_VALID_BITS_TMOS) temp_valid |= PD_CAN_TEMP_MOS_OK;
        if (c->valid & PD_VALID_BITS_TL) temp_valid |= PD_CAN_TEMP_L_OK;
        (void)pd_can_encode_temperature(c->tmos_cdeg, c->tl_cdeg, ch, temp_valid,
                                       (uint8_t)(s_meas_seq - 1u), d);
        (void)tx(PD_CAN_ID_TEMPERATURE, d);
        s_last_status = (uint16_t)((s_last_status << 1) | 1u);
    }

    /* Report a fault immediately when it appears or changes, then re-send it
       slowly so a HMI that joined late still learns about a standing fault. */
    {
        uint32_t bits = pd_can_map_fault(pd_channels[0].fault);
        uint32_t period = (bits != 0u) ? PD_CAN_FAULT_MS : 0u;
        if (bits != s_last_fault_bits || (period != 0u && (uint32_t)(now - s_last_fault) >= period))
        {
            uint8_t d[PD_CAN_DLC];
            const PD_Channel *c = &pd_channels[0];
            (void)pd_can_encode_fault(1u, bits, (uint16_t)((c->vin_mv > 0) ? c->vin_mv : 0),
                                      c->enabled ? 1u : 0u, d);
            if (tx(PD_CAN_ID_FAULT, d) == 0)
            {
                s_last_fault = now;
                s_last_fault_bits = bits;
            }
        }
    }
    flush_tx();
}

void PD_CanPortAnnounce(void)
{
    if (!s_ready)
        return;
    send_capability(1u);
}
