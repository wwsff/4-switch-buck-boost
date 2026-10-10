/**
  ******************************************************************************
  * @file    can_if.c
  * @brief   CAN transport implementation on FDCAN2 (classic CAN, 500 kbit/s).
  *
  *          The CubeMX configuration already sets:
  *            FrameFormat = FDCAN_FRAME_CLASSIC, Mode = NORMAL
  *            StdFiltersNbr = 1, TxFifoQueueElmtsNbr = 3
  *          Kernel clock is the 8 MHz HSE with NBRP = 1, Seg1 = 13, Seg2 = 2,
  *          i.e. tq = 125 ns and a 16 tq bit -> exactly 500000 bit/s.
  *          FIFO0 has 16 Message RAM entries, not just a software queue.
  ******************************************************************************
  */
#include "can_if.h"
#include "fdcan.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define RX_QUEUE_LEN 16u

static QueueHandle_t s_rx_queue;

/* --------------------------------------------------------------------------
 * Receive: the HAL calls this from the FDCAN interrupt. Only the ISR-safe API
 * may be used here - no logging, no drawing.
 * -------------------------------------------------------------------------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)hfdcan;
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
    return;
  }
  canif_rx_from_isr();
}

void canif_rx_from_isr(void)
{
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0u) {
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[CANIF_MAX_DLC];
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) {
      return;
    }
    if (hdr.IdType != FDCAN_STANDARD_ID) {
      continue;                     /* protocol uses 11-bit IDs only */
    }

    canif_frame_t f;
    f.id  = hdr.Identifier;
    f.dlc = (uint8_t)((hdr.DataLength > CANIF_MAX_DLC) ? CANIF_MAX_DLC : hdr.DataLength);
    for (uint8_t i = 0u; i < CANIF_MAX_DLC; i++) {
      f.data[i] = data[i];
    }

    BaseType_t woken = pdFALSE;
    (void)xQueueSendFromISR(s_rx_queue, &f, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* --------------------------------------------------------------------------
 * Public
 * -------------------------------------------------------------------------- */
int canif_init(void)
{
  s_rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(canif_frame_t));
  if (s_rx_queue == NULL) {
    return -1;
  }

  /* Accept every standard ID into FIFO0 and raise an interrupt per message. */
  FDCAN_FilterTypeDef filter;
  filter.IdType       = FDCAN_STANDARD_ID;
  filter.FilterIndex  = 0u;
  filter.FilterType   = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1    = 0x000u;
  filter.FilterID2    = 0x000u;      /* mask 0 = accept all */
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) {
    return -1;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return -1;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) != HAL_OK) {
    return -1;
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    return -1;
  }
  return 0;
}

int canif_receive(canif_frame_t *out, uint32_t timeout_ms)
{
  if ((out == NULL) || (s_rx_queue == NULL)) {
    return 0;
  }
  TickType_t ticks = (timeout_ms == 0u) ? 0u : pdMS_TO_TICKS(timeout_ms);
  return (xQueueReceive(s_rx_queue, out, ticks) == pdTRUE) ? 1 : 0;
}

int canif_send(uint32_t id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms)
{
  if ((data == NULL) || (dlc > CANIF_MAX_DLC)) {
    return -1;
  }

  FDCAN_TxHeaderTypeDef hdr;
  hdr.Identifier          = id;
  hdr.IdType              = FDCAN_STANDARD_ID;
  hdr.TxFrameType         = FDCAN_DATA_FRAME;
  hdr.DataLength          = (uint32_t)dlc;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch       = FDCAN_BRS_OFF;          /* classic CAN */
  hdr.FDFormat            = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker       = 0u;

  uint32_t start = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0u) {
    if ((HAL_GetTick() - start) > timeout_ms) {
      return -1;
    }
    vTaskDelay(1);
  }
  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &hdr, (uint8_t *)data) == HAL_OK) ? 0 : -1;
}
