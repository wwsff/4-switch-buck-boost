#ifndef PD_CAN_H
#define PD_CAN_H
/* CAN transport for the power board (G474). Pure encode/decode - no HAL, no
 * FreeRTOS, so the host compile gate covers it. See pd_can.c for the wire
 * format and can_protocol.h on the HMI side for the shared definitions. */
#include "powerdesk_protocol.h"
#include <stdint.h>
#define PD_CAN_ID_TEMPERATURE 0x202u
#define PD_CAN_TEMP_MOS_OK 0x01u
#define PD_CAN_TEMP_L_OK 0x02u
#define PD_CAN_TEMP_INVALID (-32768)

/* ---- Error codes reported to the HMI (0x190 byte 4) ----------------------
   These mirror CANP_ERR_* in the HMI's can_protocol.h, and the values must match
   it: the error byte goes on the wire verbatim, so a renumbered code here would
   be rendered as a different error on the panel. tools/check_protocol_agreement.py
   in CODE_USE/ enforces that.
   The board has one RANGE code where the HMI distinguishes V and I; the HMI's
   RANGE_V/RANGE_I are the finer-grained pair and the mapping is intentional. */
#define PD_CAN_ERR_NONE 0x00u
#define PD_CAN_ERR_RANGE 0x01u
#define PD_CAN_ERR_FAULT_LATCHED 0x03u
#define PD_CAN_ERR_CH_ABSENT 0x04u
#define PD_CAN_ERR_BAD_OP 0x05u
#define PD_CAN_ERR_NOT_OFF 0x07u
#define PD_CAN_ERR_NOT_READY 0x08u
#define PD_CAN_ERR_BAD_LENGTH 0x09u
#define PD_CAN_CAP_READY 0x08u
#define PD_CAN_CAP_READY_KNOWN 0x10u
#define PD_CAN_CAP_INIT_FAILED 0x20u

/* ---- Result codes (0x190 byte 3) ---------------------------------------- */
#define PD_CAN_RES_ACCEPTED 0x00u
#define PD_CAN_RES_REJECTED 0x01u
#define PD_CAN_RES_BUSY 0x02u

/* ---- Fault bitmap as the HMI wants it (0x201 bytes 2..5) ---------------- */
#define PD_CAN_FAULT_OVP 0x00000001u
#define PD_CAN_FAULT_OCP 0x00000002u
#define PD_CAN_FAULT_OTP 0x00000004u
#define PD_CAN_FAULT_OPP 0x00000008u
#define PD_CAN_FAULT_UVLO 0x00000010u
#define PD_CAN_FAULT_REVERSE 0x00000020u
#define PD_CAN_FAULT_ALERT_N 0x00000040u
#define PD_CAN_FAULT_CAN_TIMEOUT 0x00000080u
#define PD_CAN_FAULT_ADC 0x00000100u
#define PD_CAN_FAULT_INTERNAL 0x00000200u

/* ---- Flags byte on 0x201 (byte 1), mirrors CANP_FLAG_* ------------------ */
#define PD_CAN_FLAG_OUTPUT_ON 0x01u
#define PD_CAN_FLAG_FAULT_LATCHED 0x02u
#define PD_CAN_FLAG_CH2_PRESENT 0x04u

/* ---- Capability flags (0x210 byte 7), mirrors CANP_CAP_* ---------------- */
#define PD_CAN_CAP_CH_PRESENT 0x01u
#define PD_CAN_CAP_CV_CC 0x02u
#define PD_CAN_CAP_REMOTE_CLEAR 0x04u

/* Returned by pd_can_map_op() for an operation this board does not implement.
   A value the core's dispatcher cannot match, so it answers UNSUPPORTED. */
#define PD_CAN_OP_INVALID 0xFFu

/* ---- Decoded request ---------------------------------------------------- */
typedef enum
{
    PD_CAN_KIND_NONE = 0,
    PD_CAN_KIND_HEARTBEAT,
    PD_CAN_KIND_COMMAND
} pd_can_kind;

typedef struct
{
    pd_can_kind kind;
    uint16_t seq;       /* echoed back on 0x190 */
    uint8_t channel;    /* 1 or 2 */
    uint8_t can_op;     /* PD_CAN_OP_* as received */
    uint8_t op;         /* mapped PD_COMMANDS_* for the core */
    uint16_t v_mv;
    uint16_t i_ma;
    uint8_t flags;
} pd_can_request;

/* ---- API ---------------------------------------------------------------- */

/* Decode one received frame. Returns 1 when it is a frame this board accepts. */
int pd_can_decode(uint32_t id, const uint8_t *data, uint8_t dlc, pd_can_request *out);

/* Map the HMI operation code onto the core's PD_COMMANDS_* namespace. */
uint8_t pd_can_map_op(uint8_t can_op);

/* Map a core error code onto (result, error) for the HMI. Returns the error
   byte; *can_result receives PD_CAN_RES_*. */
uint8_t pd_can_map_error(uint16_t pd_error, uint8_t *can_result);

/* Core fault bitmap -> HMI fault bitmap. */
uint32_t pd_can_map_fault(uint32_t pd_fault);

/* Core state -> state the HMI displays. */
uint8_t pd_can_map_state(uint8_t pd_state, uint8_t enabled);

uint8_t pd_can_encode_temperature(int16_t mos, int16_t coil, uint8_t channel,
                                  uint8_t valid, uint8_t seq, uint8_t *out);

/* Encoders. All write exactly 8 bytes and return the DLC. */
uint8_t pd_can_encode_result(uint16_t seq, uint8_t channel, uint16_t pd_error,
                             uint16_t applied_v_mv, uint8_t state, uint8_t *out);
uint8_t pd_can_encode_measured(uint16_t v_mv, uint16_t i_ma, uint8_t channel, uint8_t state,
                               uint32_t fault, uint8_t seq, uint8_t *out);
uint8_t pd_can_encode_fault(uint8_t channel, uint32_t fault, uint16_t vin_mv, uint8_t output_on,
                            uint8_t *out);
uint8_t pd_can_encode_capability(uint8_t channel, uint16_t vmin_mv, uint16_t vmax_mv,
                                 uint16_t imax_ma, uint8_t present, uint8_t *out);

#endif
