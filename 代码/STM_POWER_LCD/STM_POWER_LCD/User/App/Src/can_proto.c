/**
  ******************************************************************************
  * @file    can_proto.c
  * @brief   Protocol v1 endpoint on the HMI side.
  *
  *          THE IMPORTANT RULE, from the plan:
  *          Output state is displayed only from the G474's measured/status
  *          report, never from a local START request or its ACK.
  ******************************************************************************
  */
#include "can_proto.h"
#include "can_if.h"
#include "can_protocol.h"
#include "bsp_log.h"
#include "main.h"        /* HAL_GetTick(); ARMCC accepted the implicit declaration,
                            GCC -Werror does not - and an implicit declaration
                            would have assumed an int return type. */

#include <string.h>

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */
typedef enum
{
  CMD_NONE = 0,
  CMD_SET,
  CMD_START,
  CMD_STOP,
  CMD_CLEAR_FAULT,
  CMD_QUERY_CAP
} cmd_kind_t;

typedef struct
{
  cmd_kind_t kind;
  uint16_t   seq;
  uint16_t   v_mv;
  uint16_t   i_ma;
  uint8_t    retries;
  uint8_t    active;
  uint32_t   last_tx_ms;
} pending_cmd_t;

static pending_cmd_t s_cmd;
static pending_cmd_t s_queue[4];
static uint8_t s_queue_count;
static uint16_t s_next_seq;
static int8_t s_set_result;
static uint16_t s_set_applied_v, s_set_applied_i;
static uint16_t      s_hb_seq;
static uint32_t      s_last_hb_ms;
static uint32_t      s_last_status_ms;
static uint8_t       s_link_was_up;
static uint8_t       s_status_seen;
static uint8_t       s_bus_ready;
static uint8_t s_temp_log_seen, s_temp_log_valid;
static uint32_t s_temp_log_ms;

static uint8_t start_allowed(void)
{
  const ui_model_t *m = ui_model();
  const ui_channel_t *c = &m->ch1;
  return (uint8_t)(m->link_up && c->installed && !c->output_on &&
                   !c->fault_latched && !c->startup_failed &&
                   (!c->ready_known || c->ready));
}

/* --------------------------------------------------------------------------
 * Packing / unpacking
 * -------------------------------------------------------------------------- */
static void send_heartbeat(void)
{
  uint8_t d[CANP_LEN_HEARTBEAT];
  ui_model_t *m = ui_model();

  d[CANP_HB_VER]   = CANP_PROTO_VERSION;
  d[CANP_HB_NODE]  = CANP_NODE_H750;
  d[CANP_HB_SEQ]   = (uint8_t)(s_hb_seq & 0xFFu);
  d[CANP_HB_FLAGS] = m->ch1.output_on ? CANP_FLAG_OUTPUT_ON : 0u;

  (void)canif_send(CANP_ID_HEARTBEAT, d, CANP_LEN_HEARTBEAT, 5u);
  s_hb_seq++;
}

static void tx_command(void)
{
  if (s_cmd.kind == CMD_START && !start_allowed()) {
    s_cmd.active = 0u;
    LOGW("CH1 ", "queued start dropped: board no longer ready");
    return;
  }
  uint8_t d[CANP_LEN_SET];
  (void)memset(d, 0, sizeof(d));

  canp_put_u16(&d[CANP_SET_SEQ], s_cmd.seq);
  d[CANP_SET_CH] = CANP_CH1;

  const char *tag = "CH1 ";
  switch (s_cmd.kind)
  {
    case CMD_SET:
      d[CANP_SET_OP] = CANP_OP_SET_SETPOINT;
      canp_put_u16(&d[CANP_SET_V_MV], s_cmd.v_mv);
      canp_put_u16(&d[CANP_SET_I_MA], s_cmd.i_ma);
      LOGI(tag, "set_request seq=%u v=%umV limit=%umA",
           (unsigned)s_cmd.seq, (unsigned)s_cmd.v_mv, (unsigned)s_cmd.i_ma);
      break;
    case CMD_START:
      d[CANP_SET_OP] = CANP_OP_START;
      LOGI(tag, "start_request seq=%u", (unsigned)s_cmd.seq);
      break;
    case CMD_STOP:
      d[CANP_SET_OP] = CANP_OP_STOP;
      LOGI(tag, "stop_request seq=%u", (unsigned)s_cmd.seq);
      break;
    case CMD_CLEAR_FAULT:
      d[CANP_SET_OP] = CANP_OP_CLEAR_FAULT;
      LOGI(tag, "clear_fault_request seq=%u", (unsigned)s_cmd.seq);
      break;
    case CMD_QUERY_CAP:
    default:
      d[CANP_SET_OP] = CANP_OP_QUERY_CAP;
      LOGI(tag, "capability_request seq=%u", (unsigned)s_cmd.seq);
      break;
  }

  if (canif_send(CANP_ID_SET, d, CANP_LEN_SET, 5u) != 0) {
    LOGE(tag, "tx_failed seq=%u", (unsigned)s_cmd.seq);
  }
  s_cmd.last_tx_ms = HAL_GetTick();
  s_cmd.retries++;
  s_cmd.active = 1u;
}

/* --------------------------------------------------------------------------
 * Receive handlers
 * -------------------------------------------------------------------------- */
static void handle_set_ack(const canif_frame_t *f)
{
  if (f->dlc < CANP_LEN_SET_ACK) {
    return;
  }
  uint16_t seq    = canp_get_u16(&f->data[CANP_ACK_SEQ]);
  uint8_t  result = f->data[CANP_ACK_RES];
  uint8_t  err    = f->data[CANP_ACK_ERR];

  if ((s_cmd.active == 0u) || (seq != s_cmd.seq) ||
      (f->data[CANP_ACK_CH] != CANP_CH1)) {
    return;                                  /* stale or unsolicited */
  }
  s_cmd.active = 0u;

  if (result == CANP_RES_ACCEPTED) {
    LOGI("CH1 ", "set_accepted seq=%u", (unsigned)seq);
    /* Reflect the value the G474 actually applied, not what we asked for. */
    if (s_cmd.kind == CMD_SET) {
      s_set_applied_v = canp_get_u16(&f->data[CANP_ACK_V_MV]);
      s_set_applied_i = s_cmd.i_ma;
      s_set_result = 1;
    }
  } else if (result == CANP_RES_BUSY) {
    LOGW("CH1 ", "set_busy seq=%u", (unsigned)seq);
    if (s_cmd.kind == CMD_SET) { s_set_result = -1; }
  } else {
    LOGE("CH1 ", "set_rejected seq=%u err=%u", (unsigned)seq, (unsigned)err);
    if (s_cmd.kind == CMD_SET) { s_set_result = -1; }
  }
  if (result != CANP_RES_ACCEPTED && s_cmd.kind == CMD_SET) {
    s_queue_count = 0u; /* never START after a rejected setpoint */
  }
}

static void handle_measured(const canif_frame_t *f)
{
  if (f->dlc < CANP_LEN_MEASURED) {
    return;
  }
  uint8_t ch = f->data[CANP_MS_CH];
  if (ch != CANP_CH1) {
    return;                                  /* CH2 not built yet */
  }

  ui_model_t   *m = ui_model();
  ui_channel_t *c = &m->ch1;

  c->v_out_mv = canp_get_u16(&f->data[CANP_MS_V_MV]);
  c->i_out_ma = canp_get_u16(&f->data[CANP_MS_I_MA]);

  uint8_t st = f->data[CANP_MS_STATE];
  /* THE display is driven from the reported state, never from our request. */
  c->run_state = st;
  c->output_on = (st == CANP_STATE_CV || st == CANP_STATE_CC ||
                  st == CANP_STATE_STARTING || st == CANP_STATE_STOPPING) ? 1u : 0u;
  c->cv_mode   = (st == CANP_STATE_CV) ? 1u : 0u;
  c->installed = (st != CANP_STATE_ABSENT) ? 1u : 0u;
  if (!c->installed) {
    c->v_out_mv = 0u;
    c->i_out_ma = 0u;
  }

  uint8_t fault_lo = f->data[CANP_MS_FAULT_LO];
  c->fault_bits = (c->fault_bits & ~0xFFu) | fault_lo;
  c->fault_latched = (c->fault_bits != 0u) ? 1u : 0u;

  s_last_status_ms = HAL_GetTick();
  s_status_seen = 1u;
  m->source = UI_SRC_CAN;
}

static void handle_fault(const canif_frame_t *f)
{
  if (f->dlc < CANP_LEN_FAULT) {
    return;
  }
  uint8_t ch = f->data[CANP_FLT_CH];
  if (ch != CANP_CH1) {
    return;
  }
  ui_model_t   *m = ui_model();
  ui_channel_t *c = &m->ch1;

  c->fault_bits    = canp_get_u32(&f->data[CANP_FLT_BITS]);
  c->fault_latched = (c->fault_bits != 0u) ? 1u : 0u;
  if (c->fault_latched != 0u) {
    c->output_on = 0u;                       /* the G474 will have stopped */
    LOGE("CH1 ", "fault=0x%08lX vout=%umV output=off",
         (unsigned long)c->fault_bits, (unsigned)c->v_out_mv);
  }
  m->source = UI_SRC_CAN;
}

static void handle_capability(const canif_frame_t *f)
{
  if (f->dlc < CANP_LEN_CAPABILITY) {
    return;
  }
  uint8_t ch = f->data[CANP_CAP_CH];
  if (ch != CANP_CH1) {
    return;
  }
  ui_model_t   *m = ui_model();
  ui_channel_t *c = &m->ch1;

  uint16_t old_vmin = c->v_min_mv, old_vmax = c->v_max_mv, old_imax = c->i_max_ma;
  uint8_t old_installed = c->installed, old_ready = c->ready, old_known = c->ready_known;
  uint8_t old_failed = c->startup_failed;
  c->v_min_mv = canp_get_u16(&f->data[CANP_CAP_VMIN_MV]);
  c->v_max_mv = canp_get_u16(&f->data[CANP_CAP_VMAX_MV]);
  uint16_t imax = canp_get_u16(&f->data[CANP_CAP_IMAX_MA]);
  if (imax > 0u) {
    c->i_max_ma = imax;
  }
  uint8_t flags = f->data[CANP_CAP_FLAGS];
  c->installed     = (flags & CANP_CAP_CH_PRESENT) ? 1u : 0u;
  c->ready_known = (flags & CANP_CAP_READY_KNOWN) ? 1u : 0u;
  c->ready = (flags & CANP_CAP_READY) ? 1u : 0u;
  c->startup_failed = (flags & CANP_CAP_INIT_FAILED) ? 1u : 0u;
  m->ch2_present   = 0u;                     /* CH2 not in this build */

  /* The UI must never hold a setpoint outside the reported range. */
  if (c->v_set_mv < c->v_min_mv) { c->v_set_mv = c->v_min_mv; }
  if (c->v_set_mv > c->v_max_mv) { c->v_set_mv = c->v_max_mv; }
  if (c->i_limit_ma > c->i_max_ma) { c->i_limit_ma = c->i_max_ma; }

  if (old_vmin != c->v_min_mv || old_vmax != c->v_max_mv || old_imax != c->i_max_ma ||
      old_installed != c->installed || old_ready != c->ready || old_known != c->ready_known ||
      old_failed != c->startup_failed) {
    LOGI("CH1 ", "capability vmin=%umV vmax=%umV imax=%umA flags=0x%02X",
         (unsigned)c->v_min_mv, (unsigned)c->v_max_mv,
         (unsigned)c->i_max_ma, (unsigned)flags);
  }
}

static void handle_temperature(const canif_frame_t *f)
{
  if (f->dlc != CANP_LEN_TEMPERATURE || f->data[CANP_TEMP_CH] != CANP_CH1 ||
      (f->data[CANP_TEMP_VALID] & ~3u) || f->data[7] != 0u) return;
  ui_channel_t *c = &ui_model()->ch1;
  uint16_t m = canp_get_u16(f->data + CANP_TEMP_MOS_CDEG);
  uint16_t l = canp_get_u16(f->data + CANP_TEMP_L_CDEG);
  c->tmos_cdeg = (int16_t)(m < 0x8000u ? (int32_t)m : (int32_t)m - 65536);
  c->tl_cdeg = (int16_t)(l < 0x8000u ? (int32_t)l : (int32_t)l - 65536);
  c->temp_valid = f->data[CANP_TEMP_VALID];
  if (c->tmos_cdeg < -4000 || c->tmos_cdeg > 12500) c->temp_valid &= ~CANP_TEMP_MOS_OK;
  if (c->tl_cdeg < -4000 || c->tl_cdeg > 12500) c->temp_valid &= ~CANP_TEMP_L_OK;
  c->temp_rx_ms = HAL_GetTick();
  c->temp_seen = 1u;
  if (!s_temp_log_seen || c->temp_valid != s_temp_log_valid ||
      (uint32_t)(c->temp_rx_ms - s_temp_log_ms) >= 2000u) {
    LOGI("TEMP", "MOS=%d L=%d cdeg valid=0x%02X", (int)c->tmos_cdeg,
         (int)c->tl_cdeg, (unsigned)c->temp_valid);
    s_temp_log_seen = 1u; s_temp_log_valid = c->temp_valid; s_temp_log_ms = c->temp_rx_ms;
  }
  /* Never set installed, link_up, output_on or main status timestamp here. */
}

static void dispatch(const canif_frame_t *f)
{
  switch (f->id)
  {
    case CANP_ID_SET_ACK:    handle_set_ack(f);    break;
    case CANP_ID_MEASURED:   handle_measured(f);   break;
    case CANP_ID_FAULT:      handle_fault(f);      break;
    case CANP_ID_TEMPERATURE: handle_temperature(f); break;
    case CANP_ID_CAPABILITY: handle_capability(f); break;
    default:                                       break;
  }
}

/* --------------------------------------------------------------------------
 * Link supervision
 * -------------------------------------------------------------------------- */
static void update_link(uint32_t now_ms)
{
  ui_model_t *m = ui_model();
  uint8_t up;

  up = (s_status_seen && (now_ms - s_last_status_ms) <= CANP_LINK_LOST_MS) ? 1u : 0u;
  if (up != s_link_was_up) {
    s_link_was_up = up;
    m->link_up = up;
    if (up != 0u) {
      LOGI("CAN ", "link_up bitrate=500000 proto=%u", (unsigned)CANP_PROTO_VERSION);
      can_proto_request_capability();
    } else {
      LOGW("CAN ", "link_lost - no status from power board");
      m->ch1.output_on = 0u;
      m->ch1.installed = 0u;
      m->ch1.ready = m->ch1.ready_known = 0u;
      m->ch1.startup_failed = 0u;
      m->ch1.temp_seen = m->ch1.temp_valid = 0u;
      m->ch1.v_out_mv = 0u;
      m->ch1.i_out_ma = 0u;
      if (s_cmd.active && s_cmd.kind == CMD_SET) { s_set_result = -1; }
      s_cmd.active = 0u;
      s_queue_count = 0u; /* never replay an old START on reconnection */
    }
  }
}

/* --------------------------------------------------------------------------
 * Public
 * -------------------------------------------------------------------------- */
int can_proto_init(void)
{
  s_bus_ready = 0u;
  s_temp_log_seen = 0u;
  (void)memset(&s_cmd, 0, sizeof(s_cmd));
  s_queue_count = 0u;
  s_next_seq = 1u;
  s_set_result = 0;
  s_hb_seq         = 0u;
  s_last_hb_ms     = 0u;
  s_last_status_ms = HAL_GetTick();
  s_link_was_up    = 0u;
  s_status_seen    = 0u;

  ui_model_t *m = ui_model();
  /* A customer-facing panel must never show simulated values as live power. */
  m->source  = UI_SRC_CAN;
  m->link_up = 0u;
  m->ch1.output_on = 0u;
  m->ch1.installed = 0u;
  m->ch1.temp_seen = m->ch1.temp_valid = 0u;
  m->ch1.v_out_mv = 0u;
  m->ch1.i_out_ma = 0u;

  if (canif_init() != 0) { return -1; }
  s_bus_ready = 1u;

  LOGI("CAN ", "bus_started bitrate=500000 proto=%u (waiting for power board)",
       (unsigned)CANP_PROTO_VERSION);
  return 0;
}

void can_proto_poll(uint32_t timeout_ms)
{
  if (!s_bus_ready) { return; } /* standalone LCD/touch still work */
  uint32_t now = HAL_GetTick();

  /* heartbeat every 250 ms */
  if ((now - s_last_hb_ms) >= CANP_HEARTBEAT_PERIOD_MS) {
    s_last_hb_ms = now;
    send_heartbeat();
  }

  canif_frame_t f;
  if (canif_receive(&f, timeout_ms) != 0) {
    dispatch(&f);
    /* drain anything else that arrived meanwhile */
    while (canif_receive(&f, 0u) != 0) {
      dispatch(&f);
    }
  }

  update_link(HAL_GetTick());
  now = HAL_GetTick();
  if (s_cmd.active && (uint32_t)(now - s_cmd.last_tx_ms) >= 150u) {
    if (s_cmd.retries < 3u && s_cmd.kind != CMD_START) {
      tx_command();
    } else {
      LOGE("CH1 ", "command_no_ack seq=%u", (unsigned)s_cmd.seq);
      if (s_cmd.kind == CMD_SET) {
        s_set_result = -1;
        s_queue_count = 0u; /* do not START with an unconfirmed target */
      }
      s_cmd.active = 0u;
    }
  }
  if (!s_cmd.active && s_queue_count > 0u) {
    s_cmd = s_queue[0];
    for (uint8_t i = 1u; i < s_queue_count; i++) { s_queue[i - 1u] = s_queue[i]; }
    s_queue_count--;
    tx_command();
  }
}

/* --------------------------------------------------------------------------
 * Command requests
 * -------------------------------------------------------------------------- */
static void queue_command(cmd_kind_t kind, uint16_t v_mv, uint16_t i_ma)
{
  pending_cmd_t next;
  (void)memset(&next, 0, sizeof(next));
  next.kind = kind;
  next.seq = s_next_seq++;
  next.v_mv = v_mv;
  next.i_ma = i_ma;
  if (kind == CMD_STOP) {
    /* A stop request supersedes all unsent work, including a queued START. */
    if (s_cmd.active && s_cmd.kind == CMD_SET) { s_set_result = -1; }
    s_queue_count = 0u;
    s_cmd = next;
    tx_command();
  } else if (s_cmd.active || s_queue_count > 0u) {
    if (s_queue_count < (uint8_t)(sizeof(s_queue) / sizeof(s_queue[0]))) {
      s_queue[s_queue_count++] = next;
    } else {
      LOGE("CAN ", "command_queue_full");
      if (kind == CMD_SET) { s_set_result = -1; }
    }
  } else {
    s_cmd = next;
    tx_command();
  }
}

void can_proto_request_set(uint16_t v_mv, uint16_t i_ma)
{
  if (!ui_model()->link_up || !ui_model()->ch1.installed) {
    s_set_result = -1;
    LOGW("CH1 ", "set_blocked: no device");
    return;
  }
  s_set_result = 0;
  queue_command(CMD_SET, v_mv, i_ma);
}

void can_proto_request_start(void)
{
  if (!start_allowed()) {
    LOGW("CH1 ", "start_blocked: offline, fault or startup not ready");
    return;
  }
  queue_command(CMD_START, 0u, 0u);
}

void can_proto_request_stop(void)
{
  queue_command(CMD_STOP, 0u, 0u);
}

void can_proto_request_clear_fault(void)
{
  queue_command(CMD_CLEAR_FAULT, 0u, 0u);
}

void can_proto_request_capability(void)
{
  queue_command(CMD_QUERY_CAP, 0u, 0u);
}

int can_proto_take_set_result(uint16_t *v_mv, uint16_t *i_ma)
{
  int result = s_set_result;
  if (result != 0) {
    if (v_mv != NULL) { *v_mv = s_set_applied_v; }
    if (i_ma != NULL) { *i_ma = s_set_applied_i; }
    s_set_result = 0;
  }
  return result;
}

uint8_t can_proto_command_pending(void)
{
  return (s_cmd.active || s_queue_count != 0u) ? 1u : 0u;
}
