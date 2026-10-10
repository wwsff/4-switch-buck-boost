#include "pd_board.h"
#include "pd_diag.h"
#include "adc.h"
#include "hrtim.h"
#include "main.h"
#include "pd_cmp.h"
#include "pd_config.h"
#include "pd_ntc.h"
#include "pd_core.h"
#include "pd_oc.h"
#include "pd_power.h"
#include "pd_zero.h"
#include "stm32g4xx_ll_adc.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define RX_SIZE 1024u
/* TX queue depth (0.3.23). The 1 Hz diagnostic burst emits SEVEN frames (codes 7,
 * 21, 23, 8, 20 and TWO rotation lines: the code-17 range snapshot and the code-25
 * target/reference pair, which the 0.3.23 field-bound split into its own frame) in the
 * SAME millisecond, and tx_poll()
 * starts only one frame per millisecond while the UART is busy (~10 ms for a
 * 115-byte LOG frame at 115200). With a depth of 4 exactly two frames were
 * dropped EVERY second - and the dropped ones were the OC line and the rotation
 * line, i.e. the code-17 board/range status that explains a refused target.
 * Measured on the bench: TXdrop grew 2.3/s and the two logs the operator sent
 * contained no code 17 at all. Depth 8 holds the whole burst with margin
 * (2.1 KB -> 4.2 KB of RAM). */
#define Q_DEPTH 8u
#if PD_OVERRIDE_NO_LATCH
/* Bench override: keep counting and logging the sampling fault, but do not stop
   the stage. PD_LoopFreeze() in the 1 kHz task still holds the duty if the
   100 kHz pair actually stops arriving. */
#define PD_ADC_FAULT_LATCH()                                                                       \
    do                                                                                             \
    {                                                                                              \
    } while (0)
#else
#define PD_ADC_FAULT_LATCH() PD_LatchFault(0, PD_FAULTS_ADC)
#endif
typedef struct
{
    uint8_t bytes[524];
    uint16_t length;
} Packet;
typedef struct
{
    Packet items[Q_DEPTH];
    uint8_t head, tail, count;
} Queue;
static Queue tx_high, tx_low;
static uint8_t tx_active[524], rx_dma[RX_SIZE];
static volatile uint32_t rx_produced, uart_errors, adc_errors;
static uint32_t rx_consumed, log_at, poll_at;
static uint16_t rx_position;
static uint16_t adc1_dma[10], adc2_dma[10], adc3_dma[4];
static volatile uint16_t slow[2][5], input_raw[2], il_raw[2];
static volatile uint32_t slow_at[2], input_at, fast_at, fast_pairs, skew_pairs, fast_cycles_max;
static uint16_t pending_il[2];
static uint8_t pending_mask, initialized;
static uint32_t pending_cycle, last_pair_cycle;
static uint32_t tx_dropped;
static uint8_t diag_index;
static uint32_t p_adc_edge_at;   /* rate limit for the ADC edge-proximity line */
static volatile uint32_t vout_acc;
/* Channel 0's last regular-group raw codes, copied out of the DMA array each 1 kHz tick BEFORE the
 * valid bits are derived from them. The gate reports these when it refuses, so the log shows the
 * inputs to the decision rather than only its outcome. */
static volatile uint16_t slow_raw[3]; /* channel 0's raw codes, for the refusal line */
/* Bumped in adc_half() every time a channel's regular group DMA half is ready. The valid bits ask
 * "did this channel deliver since the previous tick" instead of "is its age under 5 ms": a counter
 * has no comparison that can disagree with the values it is judging. */
static volatile uint32_t slow_seq[2];
static uint32_t seen_seq[2], slow_miss[2];
/* Channel 0's computed bits from the most recent tick. The refusal line prints this next to the raw
 * codes it was built from: "the bits say 0x0F but valid says 0x02" and "the bits really are 0x02"
 * are different defects, and the log could not tell them apart. */
static uint16_t ch0_bits;
static volatile uint16_t vout_acc_n;
static uint16_t vout_raw_mean;
static volatile uint32_t fault_irq_seen;
static volatile uint16_t fault_il_raw, fault_vout_raw, fault_iout_raw;
/* Which ADC timing check tripped (0.3.13): a latched PD_FAULTS_ADC alone does
   not say whether the injected pair skewed, arrived late, or the ISR overran. */
static volatile uint32_t adc_err_skew, adc_err_gap, adc_err_isr, adc_err_hal;
/* Fault debounce (see pd_config.h). adc_err_streak counts consecutive error events whose
 * spacing is under PD_ADC_ERR_STREAK_MS; adc_pair_seq proves a sample pair arrived since the
 * previous event, which is what makes a transient self-clearing. */
static volatile uint32_t adc_err_streak, adc_pair_seq;
/* Consecutive injected-IL samples above PD_IL_TRIP_MA. See the trip in pd_power.c. */
static volatile uint32_t il_over_run;
static uint32_t adc_err_at, adc_err_seq;
/* Used by every ADC error path, including the injected ISR defined above the body. ARMCC treats
 * the implicit declaration as #159 (gcc only warns -Wimplicit-function-declaration), so the
 * prototype lives with the state it maintains. */
static void adc_error_event(void);
/* Peak input-current sample since the last 1 Hz log: the 1 kHz ADC3 channel is
   the only witness to a supply that folds into current limit between logs. */
static volatile uint16_t iin_peak_raw;
/* OC_N event phase, captured from the FLT1 interrupt / flag poll. The run-length
   decision itself lives in pd_oc.c so the host suite can test it. */
static volatile uint16_t oc_mcntr, oc_cmp_in, oc_cmp_out;
static volatile uint8_t oc_pin_at_event;
static volatile uint32_t oc_flag_events;
typedef struct
{
    uint32_t fault, dt_ms;
    uint16_t vin, vout, iout, iin, il, cmp_a, cmp_b, phase;
    uint8_t state, en, gates;
} FaultSnapshot;
static volatile FaultSnapshot first_fault;
static volatile uint8_t first_fault_seen, first_fault_pending;
static uint32_t start_mark_ms;

/* Age of a sample stamped by an ISR, in milliseconds, WITHOUT the unsigned wrap.
 * adc_half() and the injected ISR stamp slow_at[]/fast_at/input_at with the LIVE HAL_GetTick(),
 * while callers compare against a HAL_GetTick() snapshot taken EARLIER in the same pass. When the
 * ISR lands in between, the stamp is newer than the caller's `now`, so `now - at` wraps to ~4.29e9
 * and reads as a 49-day-old sample. PD_PortFastAge() fixed this locally; the identical arithmetic in
 * PD_PortSamplingHealthy() gated the valid bits and refused ZERO on a maximally fresh sample, and
 * the refusal line could not show it (see the note on that gate). One idiom, applied everywhere. */
static uint32_t sample_age_ms(uint32_t now, uint32_t at)
{
    int32_t age = (int32_t)(now - at);
    return age > 0 ? (uint32_t)age : 0u;
}
/* Same age, bounded to four digits so it can be printed as %lu in a log field: the raw difference
 * is unbounded (10 digits when it wraps) and the 1 Hz ADC line carries four of them. */
static unsigned long sample_age_log(uint32_t now, uint32_t at)
{
    uint32_t age = sample_age_ms(now, at);
    return (unsigned long)(age > 999u ? 999u : age);
}
void PD_PortStartMark(void)
{
    /* Called inside the start critical section, immediately before EN. */
    start_mark_ms = HAL_GetTick();
    first_fault_seen = first_fault_pending = 0;
}
void PD_PortFaultSnapshot(uint8_t ch, uint32_t fault)
{
    /* Fixed-size copies only: no logging, waits or floating point in the ISR.
       Read cached ADC values, not ADC DR (which belongs to DMA). */
    if ((ch != 0 && ch != 1) || first_fault_seen)
        return;
    first_fault_seen = 1;
    first_fault.fault = fault;
    first_fault.dt_ms = HAL_GetTick() - start_mark_ms;
    first_fault.state = pd_channels[0].state;
    first_fault.en = (uint8_t)((GPIOA->ODR >> 3) & 1u);
    first_fault.gates = (uint8_t)(HRTIM1->sCommonRegs.OENR & 15u);
    first_fault.cmp_a = (uint16_t)HRTIM1->sTimerxRegs[0].CMP1xR;
    first_fault.cmp_b = (uint16_t)HRTIM1->sTimerxRegs[1].CMP1xR;
    first_fault.phase = (uint16_t)HRTIM1->sMasterRegs.MCNTR;
    first_fault.vin = slow[0][0];
    first_fault.vout = slow[0][1];
    first_fault.iout = slow[0][2];
    first_fault.iin = input_raw[0];
    first_fault.il = il_raw[0];
    first_fault_pending = 1;
}

uint32_t PD_PortNow(void)
{
    return HAL_GetTick();
}
uint32_t PD_PortLock(void)
{
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}
void PD_PortUnlock(uint32_t p)
{
    __set_PRIMASK(p);
}
void PD_PortDeviceID(char out[24])
{
    static const char h[] = "0123456789ABCDEF";
    uint32_t id[3];
    uint8_t i, j;
    id[0] = HAL_GetUIDw0();
    id[1] = HAL_GetUIDw1();
    id[2] = HAL_GetUIDw2();
    for (i = 0; i < 3; i++)
        for (j = 0; j < 8; j++)
            out[i * 8 + j] = h[(id[i] >> (28 - 4 * j)) & 15];
}
void PD_BoardEmergencyStop(void)
{
    if (RCC->AHB2ENR & RCC_AHB2ENR_GPIOAEN)
        GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16;
    if (RCC->AHB2ENR & RCC_AHB2ENR_GPIOBEN)
        GPIOB->BSRR = (uint32_t)GPIO_PIN_2 << 16;
    if (RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN)
        HRTIM1->sCommonRegs.ODISR = 0x00ffu;
    __DSB();
}
void PD_PortStop(uint8_t ch)
{
    if (ch == 0 || ch == 1) PD_ZeroCancel();
    if (ch == 0)
    {
        PD_BoardEmergencyStop();
        return;
    }
    if (ch == 1)
    {
        GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16;
        HRTIM1->sCommonRegs.ODISR = 0x000fu;
    }
    if (ch == 2)
    {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_2 << 16;
        HRTIM1->sCommonRegs.ODISR = 0x00f0u;
    }
    __DSB();
}
/* The ONE place that decides which hardware faults a channel may report AT ALL.
 *
 * A fault reaches PD_LatchFault() by three independent paths - the pin poll in PD_Poll(), the HRTIM
 * flag handler PD_BoardFaultIRQ(), and the software OC backstop in PD_BoardADCIRQ(). Masking only
 * some of them is exactly how ALERT_N (0.3.38) and CH2 (0.3.42) ended up "masked" while still
 * stopping the stage: the hardware action was gone, the software latch was not. Every
 * hardware-derived path funnels through here now, so a mask cannot be half-applied again.
 *
 * Bits derived from firmware decisions (OVP, the input window, COMM timeout, the ADC health gate)
 * are NOT routed through here - they are not hardware inputs and are not withdrawn. */
static uint8_t channel_fault_hw(uint8_t ch)
{
#if !PD_CH2_FITTED
    if (ch == 2)
        return 0u; /* no CH2 board fitted: its two floating pins mean nothing */
#endif
#if !PD_IL_SENSOR_FITTED
    /* The TMCS1133 supplied BOTH of CH1's hardware fault signals, not just IL. */
    if (ch == 1)
        return 0u;
#endif
    (void)ch;
    return 1u;
}
/* The HRTIM fault-interrupt enables that belong to a channel with fault hardware present.
 * Arming an interrupt for a channel whose pins float low means the flag re-asserts as fast as
 * anything clears it: that is exactly the 986-line flood 0.3.44 produced. */
static uint32_t monitored_fault_flags(void)
{
    uint32_t m = 0u;
    if (channel_fault_hw(1))
        m |= HRTIM_ISR_FLT1 | HRTIM_ISR_FLT3;
    if (channel_fault_hw(2))
        m |= HRTIM_ISR_FLT2 | HRTIM_ISR_FLT4;
    return m;
}
static uint32_t board_fault_mask(uint8_t ch)
{
    uint32_t allow = PD_FAULTS_OVERCURRENT | PD_FAULTS_SENSOR_ALERT;
    /* Same predicate the re-arm and the interrupt arming use. They MUST agree: 0.3.44 shipped them
       disagreeing and the result was a telemetry flood, not a cosmetic defect. */
    if (!channel_fault_hw(ch))
        return 0u;
#if PD_MASK_ALERT_NOUT
    allow &= (uint32_t)~PD_FAULTS_SENSOR_ALERT;
#endif
    return allow;
}
uint32_t PD_PortFaultPins(uint8_t ch)
{
    uint32_t f = 0;
    if (ch == 1)
    {
        if (!(GPIOA->IDR & GPIO_PIN_12))
            f |= PD_FAULTS_OVERCURRENT;
        if (!(GPIOB->IDR & GPIO_PIN_10))
            f |= PD_FAULTS_SENSOR_ALERT;
    }
    if (ch == 2)
    {
        if (!(GPIOA->IDR & GPIO_PIN_15))
            f |= PD_FAULTS_OVERCURRENT;
        if (!(GPIOB->IDR & GPIO_PIN_11))
            f |= PD_FAULTS_SENSOR_ALERT;
    }
#if PD_DIAG_MASK_OC
    /* 0.3.12-diag: OC_N is characterised, not obeyed. The software backstop in
       PD_BoardADCIRQ() and the slow IOUT latch still stop the stage. */
    f &= (uint32_t)~PD_FAULTS_OVERCURRENT;
#endif
    /* Applied LAST, so it also covers the diagnostic mask above and cannot be bypassed by adding
       another switch before it. */
    return f & board_fault_mask(ch);
}
void PD_PortPowerEnable(uint8_t ch, uint8_t on)
{
    if (ch == 1)
        GPIOA->BSRR = on ? (uint32_t)GPIO_PIN_3 : ((uint32_t)GPIO_PIN_3 << 16);
    if (ch == 2)
        GPIOB->BSRR = on ? (uint32_t)GPIO_PIN_2 : ((uint32_t)GPIO_PIN_2 << 16);
    __DSB();
}
void PD_PortGatesEnable(uint8_t ch, uint8_t on)
{
    if (ch == 1)
    {
        if (on)
        {
            /* Gate enable is the start of the sense-transient window measured on
               the bench; the OC_N software backstop is blanked for its duration.
               The HRTIM hardware fault action is NOT blanked, so a real
               over-current still stops the bridges immediately. */
            PD_OcArmBlank(HAL_GetTick(), PD_OC_BLANK_MS);
            HRTIM1->sCommonRegs.OENR = 0x000fu;
        }
        else
            HRTIM1->sCommonRegs.ODISR = 0x000fu;
    }
    if (ch == 2)
    {
        if (on)
            HRTIM1->sCommonRegs.OENR = 0x00f0u;
        else
            HRTIM1->sCommonRegs.ODISR = 0x00f0u;
    }
    __DSB();
}
/* Commit both bridges' CMP1xR so that the pair is loaded by ONE master update
   event.
 *
 * Hardware basis: TIMER A and TIMER B both have PreloadEnable=ENABLED,
 * UpdateTrigger1=MASTER, NumberUpdateTrigger=1, ResetUpdate=DISABLED, so each
 * timer copies its CMP1xR preload register into the active register on the
 * master timer update event. That event happens when the master counter reaches
 * PD_SW_PERIOD (and nowhere else in the period), so the pair is coherent exactly
 * when both writes finish before that same event. Masking interrupts does not
 * stop the counter - the guarantee therefore comes from the time margin
 * PD_CMP_COMMIT_MARGIN, not from the critical section.
 *
 * Sequence per attempt:
 *   1. wait for PD_CmpWindowAccepts(MCNTR) with interrupts ENABLED (bounded by
 *      PD_CMP_WAIT_GUARD visits and by PD_CMP_MAX_RETRIES periods overall);
 *   2. mask interrupts, re-read MCNTR, and only then decide. If the window
 *      closed in between, nothing has been written: retry next period.
 *   3. the masked section is measured with DWT so the log can prove the margin.
 *
 * Returns PD_CMP_OK, PD_CMP_ERR_PARAM, PD_CMP_ERR_STOPPED (fault/stop won the
 * race - nothing written) or PD_CMP_ERR_TIMEOUT (bounded retries exhausted or
 * the counter is not running). Never re-enables anything. */
uint8_t PD_PortSetCompare(uint8_t ch, uint16_t cmp_in, uint16_t cmp_out)
{
    uint32_t key, attempts = 0, guard, t0, cycles;
    uint16_t m;
    if (ch != 1 || !(RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN))
    {
        PD_CmpNoteParam();
        return PD_CMP_ERR_PARAM;
    }
    for (;;)
    {
        guard = PD_CMP_WAIT_GUARD;
        do
        {
            m = (uint16_t)HRTIM1->sMasterRegs.MCNTR;
        } while (!PD_CmpWindowAccepts(m) && --guard);
        if (!guard)
        {
            PD_CmpNoteTimeout(m, 1); /* counter stopped or gated: no window at all */
            return PD_CMP_ERR_TIMEOUT;
        }
        t0 = DWT->CYCCNT;
        key = PD_PortLock();
        m = (uint16_t)HRTIM1->sMasterRegs.MCNTR;
        if (pd_channels[0].fault)
        {
            /* Stop-priority: a fault landed while we were waiting. Write nothing
               and let the caller keep the outputs disabled. */
            PD_PortUnlock(key);
            PD_CmpNoteStopped(m);
            return PD_CMP_ERR_STOPPED;
        }
        if (PD_CmpWindowAccepts(m))
        {
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = cmp_in;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = cmp_out;
            cycles = DWT->CYCCNT - t0;
            PD_PortUnlock(key);
            PD_CmpNoteCommit(m, cycles);
            return PD_CMP_OK;
        }
        cycles = DWT->CYCCNT - t0;
        PD_PortUnlock(key);
        PD_CmpNoteRetry(m, cycles);
        if (!PD_CmpRetryAllowed(++attempts))
        {
            PD_CmpNoteTimeout(m, 0);
            return PD_CMP_ERR_TIMEOUT;
        }
    }
}
/* CMP1xR is preloaded: the value written above only reaches the compare unit at
   the next master-period update. Enabling the gate outputs before that makes
   the bridges run one or two periods at the previous (boot-time 50 %) duty,
   which drives a real current spike. Callers must wait here first and must NOT
   enable the outputs if this returns 0. */
uint8_t PD_PortWaitPeriods(uint8_t n)
{
    uint32_t last;
    uint32_t guard = 400000u;
    if (!(RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN))
    {
        PD_CmpNoteWaitTimeout();
        return 0;
    }
    last = HRTIM1->sMasterRegs.MCNTR;
    while (n && --guard)
    {
        uint32_t now = HRTIM1->sMasterRegs.MCNTR;
        if (now < last)
            n--;
        last = now;
    }
    if (n)
        PD_CmpNoteWaitTimeout();
    return (uint8_t)(n == 0);
}
/* Bridge drive mode, see pd_board.h. OUT1/OUT2 of a timer are complementary
   with dead time (DTEN=1, OUT2 sources unused), so "high side off, low side on"
   is produced by source selection alone:
     high side = OUT1 -> SET1 = NONE,      RST1 = MASTERPER  (OUT1 low, OUT2 high)
     high side = OUT2 -> SET1 = MASTERPER, RST1 = NONE       (OUT1 high, OUT2 low)
   PWM restores the compare-driven sources. Switching between the two is safe
   while the outputs are enabled: the change only takes effect at the next set
   or reset event, and the dead-time unit still separates the two gates. */
void PD_PortBridgeMode(uint8_t ch, uint8_t high_on_output2, uint8_t static_low_side)
{
    uint32_t timer = (ch == 1) ? HRTIM_TIMERINDEX_TIMER_A : HRTIM_TIMERINDEX_TIMER_B;
    if (ch != 1 && ch != 2)
        return;
    if (static_low_side)
    {
        if (high_on_output2)
        {
            HRTIM1->sTimerxRegs[timer].SETx1R = HRTIM_OUTPUTSET_MASTERPER;
            HRTIM1->sTimerxRegs[timer].RSTx1R = HRTIM_OUTPUTRESET_NONE;
        }
        else
        {
            HRTIM1->sTimerxRegs[timer].SETx1R = HRTIM_OUTPUTSET_NONE;
            HRTIM1->sTimerxRegs[timer].RSTx1R = HRTIM_OUTPUTRESET_MASTERPER;
        }
    }
    else if (high_on_output2)
    {
        HRTIM1->sTimerxRegs[timer].SETx1R = HRTIM_OUTPUTSET_TIMCMP1;
        HRTIM1->sTimerxRegs[timer].RSTx1R = HRTIM_OUTPUTRESET_MASTERPER;
    }
    else
    {
        HRTIM1->sTimerxRegs[timer].SETx1R = HRTIM_OUTPUTSET_MASTERPER;
        HRTIM1->sTimerxRegs[timer].RSTx1R = HRTIM_OUTPUTRESET_TIMCMP1;
    }
    __DSB();
}
/* Drive one whole half-bridge: the selected HRTIM output gets a 20 % narrow
   pulse and its complementary output the remaining 80 %.  The 80 % low-side
   window pulls the switch node to GND every cycle, which charges the
   UCC27301A bootstrap capacitor (the device has an internal bootstrap diode),
   so the high-side gate can actually be observed.  VIN stays interlocked below
   5 V by the caller, so no power flows. */
uint8_t PD_PortGateTestStart(uint8_t index)
{
    uint16_t narrow = (uint16_t)(PD_SW_PERIOD / 5u);                 /* 20 % */
    uint16_t wide = (uint16_t)(PD_SW_PERIOD - PD_SW_PERIOD / 5u);    /* 80 % */
    uint32_t key;
    if (index < 1 || index > 4)
        return 0;
    PD_BoardEmergencyStop();
    /* A preceding power run may have left either timer in static precharge.
       Restore PWM sources while every output is still disabled. */
    PD_PortBridgeMode(index <= 2 ? 1 : 2, 0, 0);
    if (index <= 2)
    {
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = (index == 1) ? narrow : wide;
    }
    else
    {
        HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = (index == 3) ? narrow : wide;
    }
    if (!PD_PortWaitPeriods(2))
        return 0;
    key = PD_PortLock();
    if (pd_channels[0].fault || PD_PortFaultPins(1) || !PD_PortSamplingHealthy())
    {
        PD_PortUnlock(key);
        return 0;
    }
    PD_PortStartMark();
    GPIOA->BSRR = (uint32_t)GPIO_PIN_3;
    HRTIM1->sCommonRegs.OENR = index <= 2 ? 0x0003u : 0x000cu;
    __DSB();
    PD_PortUnlock(key);
    return 1;
}
void PD_PortGateTestStop(void)
{
    HRTIM1->sCommonRegs.ODISR = 0x000fu;
    GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16;
    __DSB();
}
/* How many CONSECUTIVE injected IL samples exceeded PD_IL_TRIP_MA. The trip in pd_power.c
 * compares it against PD_IL_TRIP_COUNT, so a single coupling spike cannot stop the stage while a
 * held over-current can. */
uint32_t PD_PortIlOverRun(void)
{
    return il_over_run;
}
uint8_t PD_PortSamplingHealthy(void)
{
    uint32_t now = HAL_GetTick();
#if PD_OVERRIDE_NO_LATCH
    /* Bench override: the sticky error counter must not block a restart. Fresh
       samples are still required; PD_PowerTick stops the stage without them. */
    return (uint8_t)(initialized && fast_pairs > 100 && sample_age_ms(now, fast_at) < 3u &&
                     sample_age_ms(now, slow_at[0]) < 5u && sample_age_ms(now, slow_at[1]) < 5u &&
                     sample_age_ms(now, input_at) < 10u);
#else
    /* adc_errors is intentionally NOT tested here: it is a running total that no command can
       clear, so gating on it turned one transient into a permanent lockout. The debounced
       streak below is the equivalent protection with a recovery path. */
    return (uint8_t)(initialized && fast_pairs > 100 && sample_age_ms(now, fast_at) < 3u &&
                     sample_age_ms(now, slow_at[0]) < 5u && sample_age_ms(now, slow_at[1]) < 5u &&
                     sample_age_ms(now, input_at) < 10u && adc_err_streak < PD_ADC_ERR_STREAK);
#endif
}
/* Why is the sampling judged unhealthy? PD_PortSamplingHealthy() answers 0/1 and that single
 * bit was the whole reason the 0.3.36 field fault could not be attributed: the 1 Hz diagnostic
 * samples the counters once a second and the deciding tick falls between them. This reporter
 * names the failing sub-condition on the tick that decides, so the next occurrence is evidence
 * instead of a guess. Returns a bit mask, and fills the ages for the log line. */
uint32_t PD_PortSampleFailMask(uint32_t *fast_age, uint32_t *slow_age0, uint32_t *slow_age1,
                               uint32_t *input_age)
{
    uint32_t now = HAL_GetTick();
    uint32_t mask = 0u;
    /* Must use the SAME age definition as PD_PortSamplingHealthy(), or the mask names a cause
       the predicate did not act on - the "verdict disagrees with its own evidence" defect. */
    *fast_age = sample_age_ms(now, fast_at);
    *slow_age0 = sample_age_ms(now, slow_at[0]);
    *slow_age1 = sample_age_ms(now, slow_at[1]);
    *input_age = sample_age_ms(now, input_at);
    if (!initialized)
        mask |= 1u;                 /* not initialised yet */
    if (fast_pairs <= 100u)
        mask |= 2u;                 /* never saw 100 injected pairs */
    if (*fast_age >= 3u)
        mask |= 4u;                 /* injected pairs stale */
    if (*slow_age0 >= 5u || *slow_age1 >= 5u)
        mask |= 8u;                 /* regular group stale */
    if (*input_age >= 10u)
        mask |= 16u;                /* input-voltage rank stale */
    if (adc_err_streak >= PD_ADC_ERR_STREAK)
        mask |= 32u;                /* repeated ADC errors, debounced */
    return mask;
}
uint32_t PD_PortFastAge(void)
{
    /* Signed difference: the ISR can stamp fast_at one tick after this task read
       its own timestamp, which must not look like a 49-day stale sample. This was the one
       place that handled it; the idiom now lives in sample_age_ms() and every age test uses it. */
    if (!initialized)
        return 0u;
    return sample_age_ms(HAL_GetTick(), fast_at);
}
int PD_PortTransmit(const uint8_t *data, uint16_t n, uint8_t priority)
{
    Queue *q = priority ? &tx_high : &tx_low;
    if (n > 524 || q->count == Q_DEPTH)
    {
        tx_dropped++;
        return 0;
    }
    memcpy(q->items[q->tail].bytes, data, n);
    q->items[q->tail].length = n;
    q->tail = (q->tail + 1) % Q_DEPTH;
    q->count++;
    return 1;
}
static void tx_poll(void)
{
    Queue *q;
    uint16_t n;
    if (huart3.gState != HAL_UART_STATE_READY)
        return;
    q = tx_high.count ? &tx_high : &tx_low;
    if (!q->count)
        return;
    n = q->items[q->head].length;
    memcpy(tx_active, q->items[q->head].bytes, n);
    if (HAL_UART_Transmit_DMA(&huart3, tx_active, n) == HAL_OK)
    {
        q->head = (q->head + 1) % Q_DEPTH;
        q->count--;
    }
}
static void check(HAL_StatusTypeDef result)
{
    if (result != HAL_OK)
        Error_Handler();
}
/* Fixed settings belong to CubeMX. Verify them; never silently rewrite them. */
static void require_config(uint8_t ok)
{
    if (!ok)
        Error_Handler();
}
static void validate_adc(ADC_HandleTypeDef *a, const uint32_t channels[5], uint32_t il)
{
    static const uint32_t ranks[5] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
                                      ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5};
    static const uint32_t times[5] = {ADC_SAMPLETIME_47CYCLES_5, ADC_SAMPLETIME_47CYCLES_5,
                                      ADC_SAMPLETIME_24CYCLES_5, ADC_SAMPLETIME_92CYCLES_5,
                                      ADC_SAMPLETIME_92CYCLES_5};
    uint8_t i;
    require_config(a->Init.ClockPrescaler == ADC_CLOCK_ASYNC_DIV6 && a->Init.NbrOfConversion == 5 &&
                   a->Init.DiscontinuousConvMode == ENABLE && a->Init.NbrOfDiscConversion == 1 &&
                   a->Init.ContinuousConvMode == DISABLE &&
                   a->Init.ExternalTrigConv == ADC_EXTERNALTRIG_HRTIM_TRG1 &&
                   a->Init.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_RISING &&
                   a->Init.DMAContinuousRequests == ENABLE);
    for (i = 0; i < 5; i++)
        /* LL readback contains only the channel-number bits, whereas the
           channel literal also encodes sampling-register offsets/bitfields. */
        require_config(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_REG_GetSequencerRanks(a->Instance, ranks[i])) ==
                           __LL_ADC_CHANNEL_TO_DECIMAL_NB(channels[i]) &&
                       LL_ADC_GetChannelSamplingTime(a->Instance, channels[i]) == times[i]);
    require_config(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_INJ_GetSequencerRanks(a->Instance, ADC_INJECTED_RANK_1)) ==
                       __LL_ADC_CHANNEL_TO_DECIMAL_NB(il) &&
                   LL_ADC_GetChannelSamplingTime(a->Instance, il) == ADC_SAMPLETIME_24CYCLES_5 &&
                   LL_ADC_INJ_GetTriggerSource(a->Instance) == ADC_EXTERNALTRIGINJEC_HRTIM_TRG2 &&
                   (a->Instance->JSQR & ADC_JSQR_JL) == 0);
}
void PD_BoardInit(void)
{
    static const uint32_t ch1[5] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9,
                                    ADC_CHANNEL_3};
    static const uint32_t ch2[5] = {ADC_CHANNEL_5, ADC_CHANNEL_11, ADC_CHANNEL_17, ADC_CHANNEL_13,
                                    ADC_CHANNEL_3};
    uint32_t flags = HRTIM_ISR_FLT1 | HRTIM_ISR_FLT2 | HRTIM_ISR_FLT3 | HRTIM_ISR_FLT4;
#if PD_DIAG_MASK_OC
    char line[96];
#endif
    PD_BoardEmergencyStop();
    PD_Init();
    PD_OcInit();
    PD_CmpInit();
    /* A reset plus CubeMX initialization must already have blanking disabled. */
    require_config(
        (HRTIM1->sCommonRegs.FLTINR3 & (HRTIM_FLTINR3_FLT1BLKE | HRTIM_FLTINR3_FLT2BLKE |
                                        HRTIM_FLTINR3_FLT3BLKE | HRTIM_FLTINR3_FLT4BLKE)) == 0);
    validate_adc(&hadc1, ch1, ADC_CHANNEL_1);
    validate_adc(&hadc2, ch2, ADC_CHANNEL_2);
    require_config(hadc3.Init.ClockPrescaler == ADC_CLOCK_ASYNC_DIV6 &&
                   hadc3.Init.NbrOfConversion == 2 && hadc3.Init.DiscontinuousConvMode == DISABLE &&
                   hadc3.Init.ExternalTrigConv == ADC_EXTERNALTRIG_T7_TRGO);
    require_config(HAL_NVIC_GetPriorityGrouping() == NVIC_PRIORITYGROUP_4 &&
                   NVIC_GetPriority(ADC1_2_IRQn) == 4 && NVIC_GetPriority(HRTIM1_FLT_IRQn) == 0);
    /* Switching period and ADC scheduling (0.3.21). The expected values come
       from pd_config.h, which also holds the compile-time gates that keep the
       two Timer E triggers 50 % into a PWM period and far enough apart for the
       ADC to be idle. A CubeMX change without a matching code change must stop
       the board here instead of sampling at the wrong point of the ripple. */
    require_config(HRTIM1->sMasterRegs.MPER == PD_SW_PERIOD &&
                   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR == PD_SW_PERIOD &&
                   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR == PD_SW_PERIOD);
    /* Dead time (0.3.30). Both bridges must run the value PD_CMP_DT_COUNTS models, or the
       firmware's edge-separation and transfer models describe a stage that is not there.
       DTxR: rising in bits 5:0, falling in bits 21:16, in fHRTIM clocks (32 core counts
       each). The register is LOCKED against further writes by CubeMX (DTRLK/DTFLK), which is
       why the value is verified rather than corrected at runtime. */
    require_config((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].DTxR & 0x3Fu) ==
                       (PD_CMP_DT_COUNTS / 32u) &&
                   ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].DTxR >> 16) & 0x3Fu) ==
                       (PD_CMP_DT_COUNTS / 32u) &&
                   (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].DTxR & 0x3Fu) ==
                       (PD_CMP_DT_COUNTS / 32u) &&
                   ((HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].DTxR >> 16) & 0x3Fu) ==
                       (PD_CMP_DT_COUNTS / 32u));
    require_config(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].PERxR == PD_TE_PERIOD &&
                   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP2xR == PD_TE_CMP2 &&
                   HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP3xR == PD_TE_CMP3);
    require_config(HRTIM1->sCommonRegs.ADC1R == HRTIM_ADCTRIGGEREVENT13_TIMERE_CMP3 &&
                   HRTIM1->sCommonRegs.ADC2R == HRTIM_ADCTRIGGEREVENT24_TIMERE_CMP2 &&
                   (HRTIM1->sCommonRegs.ADCPS1 & (HRTIM_ADCPS1_AD1PSC | HRTIM_ADCPS1_AD2PSC)) == 0);
    HRTIM1->sCommonRegs.ICR = flags;
    /* Arm only the fault interrupts that belong to a channel with fault hardware present. A channel
       with no board (or whose sensor is gone) has floating pins, so its flag would re-assert as fast
       as anything clears it - see monitored_fault_flags(). */
    HRTIM1->sCommonRegs.IER |= (flags & monitored_fault_flags());
#if !PD_IL_SENSOR_FITTED
    /* TIMER A/B drive CH1 and carry FAULT1 (OC_N) + FAULT3 (ALERT_N) - see hrtim.c, where
       pTimerCfg.FaultEnable = FAULT1|FAULT3 for both. The TMCS1133 supplied both signals, so with
       it removed the flag, the interrupt and the log line all stay (they are evidence) and only the
       ACTION on the outputs goes. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].FLTxR &=
        (uint32_t)~(HRTIM_FLTR_FLT1EN | HRTIM_FLTR_FLT3EN);
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].FLTxR &=
        (uint32_t)~(HRTIM_FLTR_FLT1EN | HRTIM_FLTR_FLT3EN);
    __DSB();
#endif
#if !PD_CH2_FITTED
    /* TIMER C/D drive CH2 and carry FAULT2 + FAULT4. CH2 has no power board, so those enables were
       reacting to two floating pins. This is the hardware half of the CH2 fix; the software half is
       board_fault_mask() above. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].FLTxR &=
        (uint32_t)~(HRTIM_FLTR_FLT2EN | HRTIM_FLTR_FLT4EN);
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].FLTxR &=
        (uint32_t)~(HRTIM_FLTR_FLT2EN | HRTIM_FLTR_FLT4EN);
    __DSB();
#endif
#if PD_MASK_ALERT_NOUT && PD_IL_SENSOR_FITTED
    /* Same technique for FLT3 (ALERT_N): the flag, the interrupt and the log line stay, the output
       shutdown goes. FLT1 (OC_N) is untouched - it did not trip in the run that motivated this. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT3EN;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT3EN;
    __DSB();
#endif
#if PD_DIAG_MASK_OC
    /* Keep FLT1E (flag + interrupt) but remove its action on both CH1 bridges:
       otherwise a coupling glitch on OC_N stops the stage every time it tries to
       start, and the glitch can never be characterised. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT1EN;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT1EN;
#if PD_OVERRIDE_NO_LATCH
    /* The ALERT pin is a level-typed sensor output just like OC_N; in the bench
       override build it is logged but must not shut the bridges down either. */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT3EN;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].FLTxR &= (uint32_t)~HRTIM_FLTR_FLT3EN;
#endif
    __DSB();
#endif
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    check(HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_MASTER | HRTIM_TIMERUPDATE_A |
                                                 HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_C |
                                                 HRTIM_TIMERUPDATE_D | HRTIM_TIMERUPDATE_E));
    check(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED));
    check(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED));
    check(HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED));
    check(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma, 10));
    check(HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma, 10));
    check(HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_dma, 4));
    check(HAL_ADCEx_InjectedStart_IT(&hadc1));
    check(HAL_ADCEx_InjectedStart_IT(&hadc2));
    check(HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma, RX_SIZE));
    /* Start acquisition with EN and all eight gate outputs still disabled.
       A-D run only as timing references; there is NO WaveformOutputStart. */
    check(HAL_HRTIM_WaveformCounterStart(
        &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B |
                      HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_E));
    check(HAL_TIM_Base_Start(&htim7));
    initialized = 1;
    poll_at = HAL_GetTick();
    PD_Log(0, 1, 1, "BOOT: " PD_FW_VERSION "; CH1 restricted buck; gates off until GATE_TEST confirms");
#if PD_OVERRIDE_NO_LATCH
    /* Bench builds only; kept under 95 bytes like every other line so the tail
       is not silently cut. */
    PD_Log(0, 2, 1, "OVERRIDE BUILD: latches log-only; FLT1+FLT3 masked; comm/OVP8V/clamp/OC_N kept");
#endif
#if PD_MASK_ALERT_NOUT && !PD_OVERRIDE_NO_LATCH
    /* Say it out loud: a build with a masked protection must never be mistaken for a release one.
       Derived-free literal because the whole point is that an operator reading the log can see it. */
    PD_Log(0, 2, 1, "MONITOR BUILD: ALERT_N output shutdown masked; OC_N + IL trip + OVP active");
#endif
#if !PD_CH2_FITTED
    /* A withdrawn monitor is announced, never silent. 87 characters, inside the 95-byte field. */
    PD_Log(0, 2, 1, "CH2 NOT FITTED: fault pins float low; CH2 fault latch and log are OFF");
#endif
#if !PD_IL_SENSOR_FITTED
    /* The big one: with the TMCS1133 gone there is no hardware overcurrent trip left on CH1. */
    PD_Log(0, 2, 1, "IL SENSOR NOT FITTED: TMCS1133 removed; IL + OC_N(FLT1) + ALERT_N(FLT3) all withdrawn");
#endif
#if PD_RANGE_PROBE
    /* 86 characters, inside the 95-byte field. A probe build must not be mistaken for a normal one. */
    PD_Log(0, 2, 1, "RANGE PROBE: setpoint refusal is DISABLED; REGULATION FAILED is the only backstop left");
#endif
#if PD_DIAG_MASK_OC
    (void)snprintf(line, sizeof(line), "DIAG: FLT1 masked on A/B; OC sampled at %u kHz; supply limit is the OC backstop"
                                       "",
                   (unsigned)(PD_FAST_HZ / 1000u));
    PD_Log(0, 2, 1, line);
#endif
}
/* Report exactly which board condition blocks zeroing. `why` may be NULL for
   the per-millisecond internal check, which must not format strings. */
static uint8_t zero_check(char *why, uint32_t whysize)
{
    const PD_Channel *c = &pd_channels[0];
    uint32_t pins = PD_PortFaultPins(1);
#define ZBLOCK(...)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (why)                                                                                   \
            (void)snprintf(why, whysize, __VA_ARGS__);                                             \
        return 0;                                                                                  \
    } while (0)
    if (c->state != PD_STATES_OFF || c->enabled)
        ZBLOCK("ZERO blocked: channel not OFF (state=%u enabled=%u)", c->state, c->enabled);
    if (c->fault)
        ZBLOCK("ZERO blocked: fault latched 0x%02lX; clear fault first", (unsigned long)c->fault);
    if (c->mode != PD_MODES_NORMAL)
        ZBLOCK("ZERO blocked: mode=%u, need normal CV/CC mode", c->mode);
    if (c->vout_mv > 200)
        ZBLOCK("ZERO blocked: VOUT=%ld mV, need <=200; discharge the output",
               (long)c->vout_mv);
    if (c->vin_mv < 18000 || c->vin_mv > 22000)
        ZBLOCK("ZERO blocked: VIN=%ld mV, need 18000..22000", (long)c->vin_mv);
    if ((c->valid & (PD_VALID_BITS_VOUT | PD_VALID_BITS_VIN | PD_VALID_BITS_IOUT)) !=
        (PD_VALID_BITS_VOUT | PD_VALID_BITS_VIN | PD_VALID_BITS_IOUT))
    {
        /* Three numbers that together end the argument: what the loop COMPUTED (bits), the codes
           it computed it from (raw), and the valid bits that ended up published (v). If bits and v
           disagree, the defect is after the loop; if bits itself is missing the VIN/IOUT bits while
           the raw codes are in range, the defect is in the comparison.
           HISTORIC, because it cost a bench session: this gate used to be "now - slow_at[0] < 5",
           and `now` is snapshotted at the TOP of PD_BoardPoll() while adc_half() stamps slow_at[]
           with the LIVE tick. When the DMA half-complete landed in between, the stamp was NEWER
           than `now`, `now - slow_at[0]` wrapped to ~4.29e9, and all three bits were withheld from
           a sample that was maximally fresh. The refusal line printed a0=0 for exactly that case:
           it evaluated its OWN HAL_GetTick() inside zero_check(), hundreds of ms later, when stamp
           and clock agree again - so the wrap the gate acted on was invisible in the log, and the
           verdict appeared to contradict the raw codes. The gate is now a sequence counter
           (slow_seq[] vs seen_seq[]): no time base, nothing to wrap. m= is the miss count and is
           tested against PD_SLOW_STALE_TICKS. */
        ZBLOCK("ZERO blocked: meas invalid v=0x%04X bits=0x%02X raw=%u/%u/%u m=%u",
               (unsigned)c->valid, (unsigned)ch0_bits, (unsigned)slow_raw[0],
               (unsigned)slow_raw[1], (unsigned)slow_raw[2], (unsigned)slow_miss[0]);
    }
    if (GPIOA->ODR & GPIO_PIN_3)
        ZBLOCK("ZERO blocked: power enable PA3 still high");
    if (HRTIM1->sCommonRegs.OENR & 0x0fu)
        ZBLOCK("ZERO blocked: HRTIM outputs still enabled (OENR=0x%04lX)",
               (unsigned long)HRTIM1->sCommonRegs.OENR);
    if (pins)
        ZBLOCK("ZERO blocked: fault pin low 0x%02lX (OC_N/ALERT_N)", (unsigned long)pins);
    if (!PD_PortSamplingHealthy())
    {
        /* Counters and ages are uint32 and unbounded (49 days of uptime), so they are clamped
           before formatting: check_log_lengths.py counts every %lu at full width and an unclamped
           line would exceed the 96-byte field and be truncated on the wire - which is exactly how
           the 0.3.22 code-17 line lost the fields that explained a refused target. */
        uint32_t t = HAL_GetTick();
        ZBLOCK("ZERO blocked: sampling unhealthy adc_err=%lu streak=%lu age=%lu/%lu/%lu/%lu",
               (unsigned long)(adc_errors > 999u ? 999u : adc_errors),
               (unsigned long)(adc_err_streak > 999u ? 999u : adc_err_streak),
               sample_age_log(t, fast_at), sample_age_log(t, slow_at[0]),
               sample_age_log(t, slow_at[1]), sample_age_log(t, input_at));
    }
#undef ZBLOCK
    if (why)
        why[0] = 0;
    return 1;
}
/* The gate is consulted once per millisecond, so it may not format a string on every call - but
   it also must not stay silent when it refuses. The 14:18 ZERO failures reported only
   "allowed == false" because THIS call passed why = NULL, so the operator's log contained no
   reason at all. It now formats and logs at most about once per second. */
static uint8_t zero_allowed(void)
{
    static uint32_t refused_at;
    char why[97];
    uint32_t now = HAL_GetTick();
    if (zero_check(why, sizeof(why)))
        return 1;
    if ((uint32_t)(now - refused_at) >= 1000u)
    {
        refused_at = now ? now : 1u;
        PD_Log(1, 2, 13, why);
    }
    return 0;
}
uint16_t PD_PortZero(uint8_t action)
{
    static uint32_t last_block_log;
    char why[97];
    uint32_t now = HAL_GetTick();
    if (!action) {
        /* Query: if zeroing would be refused right now, say why. The upper
           computer's "read zero" button then doubles as a self-diagnosis. */
        if (!zero_check(why, sizeof(why)) && (uint32_t)(now - last_block_log) >= 1000u) {
            last_block_log = now;
            PD_Log(1, 2, 13, why);
        }
        return 0;
    }
    if (PD_ZeroBusy()) return PD_ERRORS_BUSY;
    if (pd_channels[0].state != PD_STATES_OFF || pd_channels[0].enabled ||
        pd_channels[0].mode != PD_MODES_NORMAL) return PD_ERRORS_NOT_OFF;
    if (!zero_check(why, sizeof(why))) {
        PD_Log(1, 2, 13, why);
        return PD_ERRORS_NOT_READY;
    }
    if (action <= 2) {
#if !PD_IL_SENSOR_FITTED
        if (action == 2) {
            /* There is no IL channel to zero. Refused explicitly rather than falling through to a
               check on a valid bit that is now never published. */
            PD_Log(1, 2, 13, "ZERO blocked: IL sensor not fitted (PD_IL_SENSOR_FITTED=0); zero the IOUT channel instead");
            return PD_ERRORS_NOT_READY;
        }
#else
        if (action == 2 && !(pd_channels[0].valid & PD_VALID_BITS_IL)) {
            PD_Log(1, 2, 13, "ZERO blocked: IL measurement invalid; check ADC1 injected sampling");
            return PD_ERRORS_NOT_READY;
        }
#endif
        PD_ZeroStart(HAL_GetTick(), action);
        PD_Log(1, 0, 13, action == 1 ? "ZERO IOUT started; keep actual current zero" :
                                      "ZERO IL started; keep actual current zero");
    } else PD_ZeroReset(action - 2);
    return 0;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *u, uint16_t pos)
{
    uint16_t delta;
    if (u != &huart3 || pos > RX_SIZE)
        return;
    delta = pos >= rx_position ? (uint16_t)(pos - rx_position)
                               : (uint16_t)(RX_SIZE - rx_position + pos);
    rx_produced += delta;
    rx_position = pos;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u)
{
    if (u == &huart3)
    {
        uart_errors |= u->ErrorCode;
        PD_BoardEmergencyStop();
    }
}
static void adc_half(ADC_HandleTypeDef *a, uint8_t full)
{
    uint8_t i, ch;
    const uint16_t *p;
    if (a == &hadc3)
    {
        p = adc3_dma + (full ? 2 : 0);
        input_raw[0] = p[0];
        input_raw[1] = p[1];
        if (p[0] > iin_peak_raw)
            iin_peak_raw = p[0];
        input_at = HAL_GetTick();
        return;
    }
    if (a == &hadc1)
    {
        ch = 0;
        p = adc1_dma + (full ? 5 : 0);
    }
    else if (a == &hadc2)
    {
        ch = 1;
        p = adc2_dma + (full ? 5 : 0);
    }
    else
        return;
    for (i = 0; i < 5; i++)
        slow[ch][i] = p[i];
    if (ch == 0)
    {
        /* One VOUT sample per regular-group sequence. At 250 kHz the trigger
           rate is 125 kHz with 5 ranks, so this accumulates at 25 kSPS and the
           1 kHz poll averages 25 samples: -25.7 dB at the 1.9 kHz LC resonance
           (the ratio f/fs is unchanged from the 20 kSPS/500 kHz design). The
           250 kHz ripple aliases to DC (250/25 = 10, exactly), i.e. it appears
           as a fixed phase-dependent offset rather than as noise - which is why
           the Timer E period must stay an integer number of PWM periods. */
        vout_acc += p[1];
        vout_acc_n++;
    }
    slow_at[ch] = HAL_GetTick();
    slow_seq[ch]++;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *a)
{
    adc_half(a, 0);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *a)
{
    adc_half(a, 1);
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *a)
{
    (void)a;
    adc_err_hal++;
    adc_error_event();
}
/* One ADC error event. Every error path calls this instead of touching adc_errors directly.
 *
 * Self-clearing: if a sample pair arrived since the previous event, the previous event did not
 * stop the sampler, so the streak restarts at 1. Otherwise the count grows, and three events
 * inside PD_ADC_ERR_STREAK_MS latch the fault for real.
 *
 * The latch is the ADC fault bit, NOT a sticky counter - so CLEAR_FAULT can release it once the
 * sampler is proved healthy again. */
static void adc_error_event(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t seq = adc_pair_seq;
    adc_errors++;
    PD_ADC_FAULT_LATCH();
    if (seq != adc_err_seq || (uint32_t)(now - adc_err_at) > PD_ADC_ERR_STREAK_MS)
        adc_err_streak = 1u;
    else
        adc_err_streak++;
    adc_err_seq = seq;
    adc_err_at = now ? now : 1u;
}
/* Called from CLEAR_FAULT before the fault bit is released. The diagnosis behind the fault has to
 * be clearable too, or the command answers NOT_READY for ever and only a power cycle recovers the
 * board. This is the recovery path the operation needs; it deliberately does NOT touch the
 * per-path counters (adc_err_skew/gap/isr/hal) or fast_cycles_max, which are evidence. */
void PD_BoardClearAdcFault(void)
{
    adc_errors = 0;
    adc_err_streak = 0u;
}
/* The IL over-run counter latches nothing itself (the fault bit does), but CLEAR_FAULT must leave
   a clean slate or the next START would trip on a stale count. */
void PD_BoardClearIlRun(void)
{
    il_over_run = 0u;
}
static int32_t current(uint16_t raw, int32_t scale)
{
    return (((int32_t)raw * PD_VREF_MV - 1650 * 4095) * scale) / 4095;
}
void PD_BoardADCIRQ(void)
{
    uint32_t entered = DWT->CYCCNT, flags = 0, elapsed;
    uint8_t i;
    if (ADC1->ISR & ADC_ISR_JEOS)
        flags |= 1;
    if (ADC2->ISR & ADC_ISR_JEOS)
        flags |= 2;
    if (flags)
    {
        if (pending_mask && ((uint32_t)(entered - pending_cycle) > 510u || (pending_mask & flags)))
        {
            pending_mask = 0;
            skew_pairs++;
            if (fast_pairs > 100)
            {
                adc_err_skew++;
                adc_error_event();
            }
        }
        if (!pending_mask)
            pending_cycle = entered;
        if (flags & 1)
        {
            pending_il[0] = (uint16_t)ADC1->JDR1;
            ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        }
        if (flags & 2)
        {
            pending_il[1] = (uint16_t)ADC2->JDR1;
            ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
        }
        pending_mask |= (uint8_t)flags;
        if (pending_mask == 3)
        {
            pending_mask = 0;
            if (last_pair_cycle && (uint32_t)(entered - last_pair_cycle) > 2550u)
            {
                skew_pairs++;
                if (fast_pairs > 100)
                {
                    adc_err_gap++;
                    adc_error_event();
                }
            }
            last_pair_cycle = entered;
            fast_at = HAL_GetTick();
            fast_pairs++;
            adc_pair_seq++;  /* proves a pair arrived; self-clears a transient error */
            for (i = 0; i < 2; i++)
            {
                il_raw[i] = pending_il[i];
#if PD_IL_TRIP_MA > 0
            /* Consecutive-sample counter for the IL trip, fed here because this is the one place
               that sees every injected sample at the full PD_FAST_HZ rate. */
            if (i == 0)
            {
                int32_t il_now = PD_ZeroCurrent(2, il_raw[0]);
                if (il_now > PD_IL_TRIP_MA)
                {
                    if (il_over_run < 255u)
                        il_over_run++;
                }
                else
                    il_over_run = 0u;
            }
#endif
                PD_FastSample(i + 1, i == 0 ? PD_ZeroCurrent(2, il_raw[i]) : current(il_raw[i], 10), pd_channels[i].vout_mv,
                              pd_channels[i].vin_mv, pd_channels[i].iout_ma,
#if PD_IL_SENSOR_FITTED
                              (uint8_t)(il_raw[i] > 8 && il_raw[i] < 4087));
#else
                              /* No IL sensor: never publish the IL valid bit, so nothing downstream
                                 can act on a floating ADC1_IN1. The injected group keeps running - it
                                 is what feeds fast_pairs/fast_at, and PD_PortSamplingHealthy()
                                 requires those, so removing the group would stop the stage. */
                              (uint8_t)0);
#endif
            }
#if PD_DIAG_MASK_OC
            /* Poll the FLT1 status flag as well: the interrupt is one-shot, so
               a second event inside the same second would otherwise be missed.
               The ISR clears the flag when it does fire, so no double count. */
            if (HRTIM1->sCommonRegs.ISR & HRTIM_ISR_FLT1)
            {
                oc_flag_events++;
                oc_mcntr = (uint16_t)HRTIM1->sMasterRegs.MCNTR;
                oc_cmp_in = HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
                oc_cmp_out = HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR;
                oc_pin_at_event = (uint8_t)((GPIOA->IDR & GPIO_PIN_12) ? 1u : 0u);
                HRTIM1->sCommonRegs.ICR = HRTIM_ISR_FLT1;
            }
            /* OC_N sampled once per 10 us pair; pd_oc.c turns the low-run length
               into a stop decision (blanked at gate enable). */
#if PD_IL_SENSOR_FITTED
            if (PD_OcFeed((uint8_t)((GPIOA->IDR & GPIO_PIN_12) ? 0u : 1u), HAL_GetTick()))
            {
                /* Sustained OC outside the blank window: stop for real. Logged
                   from the 1 kHz task, never from here. */
                PD_BoardEmergencyStop();
                PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
            }
#else
            /* PA12 is OC_N, a TMCS1133 output. With the sensor removed this pin floats, and the
               "sustained low run" this backstop looks for is exactly what a floating input would
               produce - it would stop the stage on nothing. The backstop existed to cover OC_N's
               masked hardware action; with no OC_N at all there is nothing for it to cover. */
#endif
#endif
        }
    }
    /* Only exceptional regular/injected queue errors take the HAL path. */
    if (ADC1->ISR & (ADC_ISR_OVR | ADC_ISR_JQOVF))
        HAL_ADC_IRQHandler(&hadc1);
    if (ADC2->ISR & (ADC_ISR_OVR | ADC_ISR_JQOVF))
        HAL_ADC_IRQHandler(&hadc2);
    elapsed = DWT->CYCCNT - entered;
    if (elapsed > fast_cycles_max)
        fast_cycles_max = elapsed;
    if (elapsed >= 1700u && fast_pairs > 100)
    {
        adc_err_isr++;
        adc_error_event();
    }
}
void PD_BoardFaultIRQ(void)
{
    uint32_t raw = HRTIM1->sCommonRegs.ISR &
                   (HRTIM_ISR_FLT1 | HRTIM_ISR_FLT2 | HRTIM_ISR_FLT3 | HRTIM_ISR_FLT4);
    uint32_t f;
    if (!raw)
        return;
    /* Only flags whose interrupt is armed belong to this channel's event; the
       other bits are stale from a pin that is still low (CH2 has no board). */
    f = raw & HRTIM1->sCommonRegs.IER;
    if (f)
    {
        fault_irq_seen |= f; /* logged from the 1 kHz task, never from this ISR */
        fault_il_raw = il_raw[0];
        fault_vout_raw = slow[0][1];
        fault_iout_raw = slow[0][2];
    }
#if PD_DIAG_MASK_OC
    if (f & HRTIM_ISR_FLT1)
    {
        /* Characterise the OC_N event: how often, and at which phase of the
           switching period (MCNTR 0..PER, CMP1_A = input high-side on-time).
           The interrupt is deliberately ONE-SHOT: a level-sensitive fault flag
           re-enters immediately while OC_N stays low, and that storm starved
           the 100 kHz sampling ISR (observed on the bench as a latched
           PD_FAULTS_ADC). Re-armed by the 1 kHz task only while OC_N is high;
           the 100 kHz poll keeps counting the rest. */
        oc_flag_events++;
        oc_mcntr = (uint16_t)HRTIM1->sMasterRegs.MCNTR;
        oc_cmp_in = HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
        oc_cmp_out = HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR;
        oc_pin_at_event = (uint8_t)((GPIOA->IDR & GPIO_PIN_12) ? 1u : 0u);
        HRTIM1->sCommonRegs.IER &= (uint32_t)~HRTIM_ISR_FLT1;
        HRTIM1->sCommonRegs.ICR = HRTIM_ISR_FLT1;
        f &= (uint32_t)~HRTIM_ISR_FLT1;
        raw &= (uint32_t)~HRTIM_ISR_FLT1;
        if (!f)
            return;
    }
#endif
#if !PD_OVERRIDE_NO_LATCH
    /* The flags and the log line are kept as evidence; only the LATCH is filtered. Filtering the
       bit word here rather than guarding each branch is what makes board_fault_mask() complete -
       this is the path that made the 0.3.38 ALERT_N mask and the 0.3.42 CH2 mask ineffective. */
    if (f & (HRTIM_ISR_FLT1 | HRTIM_ISR_FLT3))
    {
        uint32_t bits = ((f & HRTIM_ISR_FLT1) ? PD_FAULTS_OVERCURRENT : 0u) |
                        ((f & HRTIM_ISR_FLT3) ? PD_FAULTS_SENSOR_ALERT : 0u);
        bits &= board_fault_mask(1);
        if (bits)
            PD_LatchFault(1, bits);
    }
    if (f & (HRTIM_ISR_FLT2 | HRTIM_ISR_FLT4))
    {
        uint32_t bits = ((f & HRTIM_ISR_FLT2) ? PD_FAULTS_OVERCURRENT : 0u) |
                        ((f & HRTIM_ISR_FLT4) ? PD_FAULTS_SENSOR_ALERT : 0u);
        bits &= board_fault_mask(2);
        if (bits)
            PD_LatchFault(2, bits);
    }
#endif
    HRTIM1->sCommonRegs.IER &= ~f;
    HRTIM1->sCommonRegs.ICR = raw;
}
/* Filter only plausible fresh readings; an open wire must invalidate at once.
 * A new valid reading initializes the filter, so cold boot never ramps from 0 C. */
static int32_t ntc_filter[2][2];
static uint8_t ntc_filter_valid[2][2];
static int16_t ntc_value[2][2];
static uint32_t ntc_at;
static uint8_t ntc_started;
static int16_t ntc_read(uint8_t ch, uint8_t sensor, uint16_t raw, uint8_t fresh, uint8_t due)
{
    int16_t value = (raw >= PD_NTC_RAW_MIN && raw <= PD_NTC_RAW_MAX) ? 0 : PD_NTC_INVALID;
    if (!fresh || value == PD_NTC_INVALID) {
        ntc_filter_valid[ch][sensor] = 0u;
        ntc_value[ch][sensor] = PD_NTC_INVALID;
    } else if (due || !ntc_filter_valid[ch][sensor]) {
        int32_t input = (int32_t)raw * 256;
        if (!ntc_filter_valid[ch][sensor]) ntc_filter[ch][sensor] = input;
        else ntc_filter[ch][sensor] += (input - ntc_filter[ch][sensor]) / 4;
        ntc_filter_valid[ch][sensor] = 1u;
        ntc_value[ch][sensor] = PD_NtcConvert((uint16_t)((ntc_filter[ch][sensor] + 128) / 256));
    }
    return ntc_value[ch][sensor];
}
void PD_BoardPoll(void)
{
    uint32_t now = HAL_GetTick(), available, key, mask, vacc;
    uint16_t raw[2][5], in[2], vn;
    uint8_t i, j, ntc_due;
    char line[96];
    FaultSnapshot snapshot;
    if (!initialized)
        return;
    if (first_fault_pending)
    {
        key = PD_PortLock();
        snapshot = first_fault;
        PD_PortUnlock(key);
        if (first_fault_pending == 1)
        {
            /* Two frames on purpose: with the duty pair folded in, this line was
               114 characters at its widest and lost its tail on the wire (the
               defect this version fixes). */
            (void)snprintf(line, sizeof(line),
                       "FSTOP f=%02lX st=%u EN=%u g=%X t=%lu ms ph=%u",
                       (unsigned long)snapshot.fault, snapshot.state, snapshot.en, snapshot.gates,
                       (unsigned long)snapshot.dt_ms, snapshot.phase);
            if (PD_TryLog(1, 2, 22, line))
                first_fault_pending = 2;
        }
        if (first_fault_pending == 2)
        {
            /* The duty pair is printed as raw/32 so %u cannot exceed four digits. */
            /* Raw counts, NOT PD_LOG_VALUE: the duty pair is a register value in
               0..PD_SW_PERIOD, and the 999 cap would print "1000" for every real
               operating point - i.e. the line would say nothing. Five digits is what
               the format's %u already allows and it fits the field. */
            (void)snprintf(line, sizeof(line), "FSTOP duty A=%u B=%u of %u ph=%u",
                       snapshot.cmp_a, snapshot.cmp_b, (unsigned)PD_SW_PERIOD, snapshot.phase);
            if (PD_TryLog(1, 2, 22, line))
                first_fault_pending = 0;
        }
    }
    if (uart_errors)
    {
        uint32_t error = uart_errors;
        uart_errors = 0;
        PD_LatchFault(0, PD_FAULTS_COMM_TIMEOUT);
        (void)HAL_UART_AbortReceive(&huart3);
        rx_position = 0;
        rx_produced = rx_consumed = 0;
        PD_ResetReceiver();
        check(HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma, RX_SIZE));
        (void)snprintf(line, sizeof(line), "UART error=0x%lx; RX restarted", (unsigned long)error);
        PD_Log(0, 2, 5, line);
    }
    available = rx_produced - rx_consumed;
    if (available > RX_SIZE)
    {
        rx_consumed = rx_produced;
        PD_ResetReceiver();
        PD_LatchFault(0, PD_FAULTS_COMM_TIMEOUT);
        PD_Log(0, 2, 6, "RX DMA overrun; unread data discarded");
    }
    else
        while (available--)
        {
            uint8_t b = rx_dma[rx_consumed % RX_SIZE];
            rx_consumed++;
            PD_Feed(&b, 1);
        }
    if (now != poll_at)
    {
        poll_at = now;
        ntc_due = (uint8_t)(!ntc_started || (uint32_t)(now - ntc_at) >= PD_NTC_PERIOD_MS);
        if (ntc_due) { ntc_at = now; ntc_started = 1u; }
        key = PD_PortLock();
        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < 5; j++)
                raw[i][j] = slow[i][j];
            in[i] = input_raw[i];
        }
        vacc = vout_acc;
        vn = vout_acc_n;
        vout_acc = 0;
        vout_acc_n = 0;
        PD_PortUnlock(key);
        if (vn)
            vout_raw_mean = (uint16_t)((vacc + (uint32_t)vn / 2u) / vn);
        for (i = 0; i < 2; i++)
        {
            /* Voltage channels use the calibrated conversion (0.3.22): measured
               reference 3285 mV plus the per-channel gain/offset hook. VOUT for
               CH1 stays the 25-sample sequence average that vout_raw_mean holds;
               the current channels keep their old basis (PD_VREF_MV + the
               pd_zero.c zero store) on purpose. */
            int32_t vin = PD_ConvertVinMv(raw[i][0]);
            int32_t vout = PD_ConvertVoutMv(i == 0 ? vout_raw_mean : raw[i][1]);
            uint16_t bits = 0;
            if (i == 0)
            {
                slow_raw[0] = raw[0][0];
                slow_raw[1] = raw[0][1];
                slow_raw[2] = raw[0][2];
            }
            /* FRESH = "this channel delivered since the previous tick", proven by slow_seq[].
               Replaces "now - slow_at[i] < 5", which reported all three bits absent while the
               codes it judges (raw = 2252/3/2037) were inside every range it tests and its own
               age read 0 ms. A channel that really stops still loses its bits, but after
               PD_SLOW_STALE_TICKS ticks, so a stuck conversion is never published as a reading. */
            if ((uint32_t)(slow_seq[i] - seen_seq[i]) != 0u)
            {
                seen_seq[i] = slow_seq[i];
                slow_miss[i] = 0u;
            }
            else if (slow_miss[i] < 255u)
                slow_miss[i]++;
            if (slow_miss[i] < PD_SLOW_STALE_TICKS)
            {
                if (raw[i][0] < 4087)
                    bits |= PD_VALID_BITS_VIN;
                if (raw[i][1] < 4087)
                    bits |= PD_VALID_BITS_VOUT;
                if (raw[i][2] > 8 && raw[i][2] < 4087)
                    bits |= PD_VALID_BITS_IOUT;
            }
            if (i == 0)
                ch0_bits = bits; /* what THIS loop computed, for the refusal line */
            if (sample_age_ms(now, input_at) < 10u && in[i] > 8 && in[i] < 4087)
                bits |= PD_VALID_BITS_IIN;
            /* Regular rank 4/5 = MOS/inductor (PC3/PA2, PA5/PA6).
             * Temperature failure is reported, but does not change the existing
             * power interlocks: thermal protection still needs bench approval. */
            uint8_t temp_fresh = (uint8_t)(slow_miss[i] < PD_SLOW_STALE_TICKS &&
                                          adc_err_streak < PD_ADC_ERR_STREAK &&
                                          (i == 0u || PD_CH2_FITTED));
            int16_t tm = ntc_read(i, 0u, raw[i][3], temp_fresh, ntc_due);
            int16_t tl = ntc_read(i, 1u, raw[i][4], temp_fresh, ntc_due);
            if (tm != PD_NTC_INVALID) bits |= PD_VALID_BITS_TMOS;
            if (tl != PD_NTC_INVALID) bits |= PD_VALID_BITS_TL;
            key = PD_PortLock();
            pd_channels[i].tmos_cdeg = tm;
            pd_channels[i].tl_cdeg = tl;
            pd_channels[i].vin_mv = vin;
            pd_channels[i].vout_mv = vout;
            pd_channels[i].iout_ma = i == 0 ? PD_ZeroCurrent(1, raw[i][2]) : current(raw[i][2], 5);
            pd_channels[i].iin_ma = current(in[i], 10);
            pd_channels[i].valid = (pd_channels[i].valid & PD_VALID_BITS_IL) | bits;
            PD_PortUnlock(key);
            /* channel_fault_hw() FIRST, and it is not optional. Before 0.3.44 the stuck-low pins
               made PD_PortFaultPins() non-zero here and that suppressed this re-arm by accident;
               masking the report turned the condition TRUE and the interrupts were re-armed every
               millisecond against pins that were still low. 986 log lines in two minutes. */
            if (channel_fault_hw(i + 1) && !PD_PortFaultPins(i + 1) &&
                pd_channels[i].state == PD_STATES_OFF)
            {
                mask =
                    i == 0 ? (HRTIM_ISR_FLT1 | HRTIM_ISR_FLT3) : (HRTIM_ISR_FLT2 | HRTIM_ISR_FLT4);
                if ((HRTIM1->sCommonRegs.IER & mask) != mask)
                {
                    HRTIM1->sCommonRegs.ICR = mask;
                    HRTIM1->sCommonRegs.IER |= mask;
                }
            }
        }
        {
            uint8_t event = PD_ZeroStartupTick(now, zero_check(NULL, 0u));
            if (event == PD_ZERO_BOOT_WAIT_EVENT)
                PD_Log(1, 0, 13, "AUTOZERO waiting: output OFF, VIN 18..22 V, VOUT <=200 mV, healthy sampling");
            else if (event == PD_ZERO_BOOT_START_EVENT)
                PD_Log(1, 0, 13, "AUTOZERO IOUT started; output stays OFF");
            else if (event == PD_ZERO_BOOT_READY_EVENT)
                PD_Log(1, 0, 13, "AUTOZERO IOUT ready; output may start if other checks pass");
            else if (event == PD_ZERO_BOOT_FAILED_EVENT)
                PD_Log(1, 2, 13, "AUTOZERO IOUT failed; output remains locked (see ZERO status/log)");
        }
        if (PD_ZeroBusy())
        {
            PD_ZeroStatus zero;
            PD_ZeroGet(&zero);
            PD_ZeroFeed(now, zero_allowed() && (zero.sensor != 2 ||
                        (pd_channels[0].valid & PD_VALID_BITS_IL)), 1,
                        zero.sensor == 1 ? raw[0][2] : il_raw[0]);
            if (!PD_ZeroBusy()) {
                PD_ZeroGet(&zero);
                (void)snprintf(line, sizeof(line), "ZERO st=%u sen=%u why=%u n=%u IOUT=%lduV IL=%lduV",
                    zero.state, zero.sensor, zero.reason, zero.samples,
                    (unsigned long)zero.iout_uv, (unsigned long)zero.il_uv);
                PD_Log(1, zero.state == 3 ? 0 : 1, 13, line);
            }
        }
        if (now > 100 && !PD_PortSamplingHealthy())
        {
            PD_ADC_FAULT_LATCH();
            if (sample_age_ms(now, fast_at) > 3u)
            {
                key = PD_PortLock();
                pd_channels[0].valid &= (uint16_t)~PD_VALID_BITS_IL;
                pd_channels[1].valid &= (uint16_t)~PD_VALID_BITS_IL;
                PD_PortUnlock(key);
            }
        }
        if (fault_irq_seen)
        {
            uint32_t f = fault_irq_seen;
            fault_irq_seen = 0;
            (void)snprintf(line, sizeof(line),
                           "HRTIM FLT m=0x%02lX (1=CH1OC 2=CH2OC 4=CH1ALR 8=CH2ALR) scan=%u/%u/%u pin=%02lX"
                           "",
                           (unsigned long)f, fault_il_raw, fault_vout_raw, fault_iout_raw,
                           (unsigned long)PD_PortFaultPins(1));
            PD_Log(0, 2, 18, line);
        }
        PD_GateTestTick();
        PD_PowerTick();
        HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin,
                          (now / 500) % 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin,
                          (pd_channels[0].fault || pd_channels[1].fault) ? GPIO_PIN_SET
                                                                         : GPIO_PIN_RESET);
    }
    PD_Poll();
    tx_poll();
    if (now - log_at >= 1000)
    {
        log_at = now;
        key = PD_PortLock();
        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < 5; j++)
                raw[i][j] = slow[i][j];
            in[i] = input_raw[i];
        }
        PD_PortUnlock(key);
        (void)snprintf(
            line, sizeof(line), "ADC fast=%lu skew=%lu mx=%lu ce=%lu TXd=%lu ag=%lu/%lu/%lu/%lu",
            PD_LOG_VALUE(fast_pairs), PD_LOG_VALUE(skew_pairs), PD_LOG_VALUE(fast_cycles_max),
            PD_LOG_VALUE(adc_errors), PD_LOG_VALUE(tx_dropped), sample_age_log(now, fast_at),
            sample_age_log(now, slow_at[0]), sample_age_log(now, slow_at[1]),
            sample_age_log(now, input_at));
        PD_Log(0, 0, 7, line);
        (void)snprintf(line, sizeof(line), "ADCERR s/g/i/h=%lu/%lu/%lu/%lu (skew/gap/ISR/HAL)",
                       (unsigned long)adc_err_skew, (unsigned long)adc_err_gap,
                       (unsigned long)adc_err_isr, (unsigned long)adc_err_hal);
        PD_Log(0, adc_errors ? 2 : 0, 21, line);
        /* Compare-commit statistics (0.3.19): reason codes, bounded retries and
           the measured worst-case masked section. Formatted here in the 1 kHz
           task, never inside the critical section or an ISR. */
        (void)snprintf(line, sizeof(line),
                       "CMP ok=%lu retry=%lu to=%lu stop=%lu par=%lu gd=%lu wt=%lu stg=%u rsn=%u M=%u",
                       PD_LOG_VALUE(PD_CmpCommits()), PD_LOG_VALUE(PD_CmpRetries()),
                       PD_LOG_VALUE(PD_CmpTimeouts()), PD_LOG_VALUE(PD_CmpStopped()),
                       PD_LOG_VALUE(PD_CmpParamErrors()), PD_LOG_VALUE(PD_CmpGuardExpired()),
                       PD_LOG_VALUE(PD_CmpWaitTimeouts()), PD_CmpStage(), PD_CmpLastReason(),
                       PD_LOG_VALUE(PD_CmpLastCounter()));
        PD_Log(0, (PD_CmpTimeouts() || PD_CmpParamErrors() || PD_CmpStopped()) ? 2 : 0, 23, line);
        key = PD_PortLock();
        vn = iin_peak_raw;
        iin_peak_raw = 0;
        PD_PortUnlock(key);
        (void)snprintf(line, sizeof(line), "TEMP MOS=%d L=%d cdeg valid=%02X raw=%u/%u",
                       pd_channels[0].tmos_cdeg, pd_channels[0].tl_cdeg,
                       (unsigned)(pd_channels[0].valid & (PD_VALID_BITS_TMOS | PD_VALID_BITS_TL)),
                       slow[0][3], slow[0][4]);
        PD_Log(1, 0, 24, line);
        (void)snprintf(line, sizeof(line),
                       "CH1 raw IL=%u VIN=%u VO=%u IO=%u TM=%u TL=%u IN=%u pk=%ldmV",
                       il_raw[0], raw[0][0], raw[0][1], raw[0][2], raw[0][3], raw[0][4], in[0],
                       (long)current(vn, 10));
        PD_Log(1, 0, 8, line);
#if PD_DIAG_MASK_OC
        /* Re-arm the one-shot FLT1 interrupt only while OC_N is high. */
        if (channel_fault_hw(1) && (GPIOA->IDR & GPIO_PIN_12) &&
            !(HRTIM1->sCommonRegs.IER & HRTIM_ISR_FLT1))
            HRTIM1->sCommonRegs.IER |= HRTIM_ISR_FLT1;
#endif
        (void)snprintf(line, sizeof(line),
                       "OC ev=%lu blk=%lu low=%lu runs=%lu max=%lu bs=%lu M=%u A=%u B=%u p=%u",
                       PD_LOG_VALUE(oc_flag_events), PD_LOG_VALUE(PD_OcBlankedSamples()),
                       PD_LOG_VALUE(PD_OcLowSamples()), PD_LOG_VALUE(PD_OcRuns()),
                       PD_LOG_VALUE(PD_OcMaxRun()), PD_LOG_VALUE(PD_OcTrips()),
                       PD_LOG_VALUE(oc_mcntr / 32u), PD_LOG_VALUE(oc_cmp_in / 32u),
                       PD_LOG_VALUE(oc_cmp_out / 32u), oc_pin_at_event);
        PD_Log(0, 0, 20, line);
        if (diag_index == 0)
        {
            /* States which calibration basis each channel uses and whether the
               gain/offset hook is still inert (cal=0 means PD_CALIBRATED=0, i.e. no
               full-channel calibration). Width: tools/check_log_lengths.py - a
               hand-written number here went stale once already. */
            (void)snprintf(line, sizeof(line),
                           "CONFIG MODER=%08lx ID=%08lx Vs=%u/%u cal=%u gain=%u/%u off=%ld",
                           (unsigned long)GPIOC->MODER, (unsigned long)DBGMCU->IDCODE,
                           (unsigned)PD_VOLTAGE_SCALE_REF_MV, (unsigned)PD_VREF_MV,
                           (unsigned)PD_CALIBRATED, (long)PD_VOUT_CAL_GAIN_NUM,
                           (long)PD_VOUT_CAL_GAIN_DEN, (long)PD_VOUT_CAL_OFFSET_MV);
            PD_Log(1, 0, 9, line);
        }
        else if (diag_index == 1)
        {
            (void)snprintf(line, sizeof(line), "ADC1 SQR1=%08lx SQR2=%08lx CFGR=%08lx JSQR=%08lx",
                           (unsigned long)ADC1->SQR1, (unsigned long)ADC1->SQR2,
                           (unsigned long)ADC1->CFGR, (unsigned long)ADC1->JSQR);
            PD_Log(1, 0, 10, line);
        }
        else if (diag_index == 2)
        {
            (void)snprintf(line, sizeof(line),
                           "UART RX=%lu con=%lu pos=%u ISR=%08lx CR3=%08lx",
                           (unsigned long)rx_produced, (unsigned long)rx_consumed, rx_position,
                           (unsigned long)USART3->ISR, (unsigned long)USART3->CR3);
            PD_Log(0, 0, 11, line);
        }
        else if (diag_index == 4)
        {
            /* Control-term frame (0.3.25 step 1, extended in 0.3.28). The voltage loop computes
               these every 1 kHz tick; this exports them so the bench can SHOW whether the
               integrator is pinned on its clamp instead of reverse-engineering it from
               ref/vout/duty. sat=3 means exactly that, in either direction. q/b/lim were added
               when the actuator became a bridge PAIR: with those three the log answers "what did
               the loop ask the modulator for, what did the registers get, and what stopped it",
               which is the whole of the phase-A question. */
            PD_CtrlTerms t;
            PD_PowerCtrlTerms(&t);
            (void)PD_DiagControlLine(line, sizeof(line), t.ref_mv, t.vout_f_mv, t.ff_cnt,
                                     t.p_cnt, t.i_cnt, t.q_cnt, t.cmd_cnt, t.q_app_cnt,
                                     t.cmp_out_cnt, t.mod_limit, t.sat);
            PD_Log(1, 0, 26, line);
            /* ADC edge proximity (0.3.32). Reported next to the control terms because it is the
               same question from the other side: the trace says what the loop asked for, this
               says whether the measurement it is regulating on was taken on a commutation edge.
               Rate-limited to one line per PD_ADC_EDGE_WATCH_MS so the 1 Hz budget is untouched,
               and only emitted while the condition actually holds. */
            {
                int32_t dist = 0;
                uint8_t near_edge = PD_PowerAdcEdgeNear(pd_channels[0].duty_in,
                                                        pd_channels[0].duty_out, &dist);
                if (near_edge && now - p_adc_edge_at >= PD_ADC_EDGE_WATCH_MS)
                {
                    p_adc_edge_at = now ? now : 1u;
                    (void)PD_DiagSamplingLine(line, sizeof(line), pd_channels[0].duty_in,
                                              pd_channels[0].duty_out,
                                              (uint16_t)(PD_TE_CMP3 % PD_SW_PERIOD),
                                              dist, near_edge);
                    PD_Log(1, 2, 15, line);
                }
            }
        }
        else if (diag_index == 3)
        {
            /* What the loop is being TOLD, kept in the rotation on purpose (0.3.23):
               a target sitting just above what the measured VIN allows settles with an
               error inside the PD_REG_FAIL_MS margin, so nothing is logged and nothing
               trips - 17 V was silently clipped exactly that way at the 20 V input. With
               tgt/ref in the 1 Hz stream, "ref is not following tgt" is visible before
               it becomes a fault. It is a separate frame because the range snapshot
               cannot carry eight measured fields and these three inside 95 characters.
                 tgt  accepted target (mV)      ref  reference the loop executes (mV)
                 ovp  OVP threshold in force (mV)   dn  1 while a descent is supervised */
            (void)PD_DiagTargetLine(line, sizeof(line), PD_PowerTargetMv(),
                                    PD_PowerRampMv(), PD_PowerOvpLimitMv(),
                                    PD_PowerDownActive());
            PD_Log(1, 0, 25, line);
        }
        else
        {
            /* Range/saturation snapshot (0.3.23). The channel is already in the
               frame header, so the line starts with values, and the target /
               reference / OVP triple is a separate frame (code 25) because eight
               measured fields do not fit the 96-byte field at their type widths.
               The 0.3.22 version of this line was ~140 characters and silently
               lost its tail - exactly the VIN/vmax fields needed to explain a
               refused target - so the redundant prefix and md=/vld=/EN=/OENR=/
               pin=/en= were dropped (FSTOP carries those on any fault, and
               `enabled` is implied by the state). Width and margin:
               tools/check_log_lengths.py / dump_log_lines.py --all.
                 s    state            f   fault bits        D  A/B compare values
                 S    1 = the committed pair is at the END of its usable range
                      (PD_PowerAtRangeEnd: q at the ceiling in linked mode, the A
                      bridge at PD_CMP_A_MAX in legacy). It is NOT "A is at 70 %":
                      in linked mode that happens at q = 0.75 with 15 percentage
                      points of B travel still in hand, and the 2026-09-13 bench run
                      at 18 V out of 19.82 V in sat exactly there.
                 IO/VO measured output current/voltage
                 VI/vm measured VIN and the output range it allows (mV) */
            (void)PD_DiagStatusLine(line, sizeof(line), pd_channels[0].state,
                                    (uint32_t)pd_channels[0].fault, pd_channels[0].duty_in,
                                    pd_channels[0].duty_out,
                                    PD_PowerAtRangeEnd(),
                                    pd_channels[0].iout_ma, pd_channels[0].vout_mv,
                                    pd_channels[0].vin_mv,
                                    PD_PowerAllowedVmaxMv(pd_channels[0].vin_mv));
            PD_Log(1, 0, 17, line);
        }
        /* Six slots now: CONFIG, ADC1, UART, the range snapshot, the target/OVP frame and the
           control-term frame. Two rotation lines appear per second (diag_index advances one
           per tick), so the control terms are at most 2 s old - enough to watch an
           integrator, and it keeps the 1 Hz burst at seven frames, inside Q_DEPTH 8. */
        diag_index = (uint8_t)((diag_index + 1) % 6);
    }
}
