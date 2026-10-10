#ifndef PD_CONFIG_H
#define PD_CONFIG_H
/* User-confirmed first bench: 20 V +/-1%, CH1 1..30 V / <=2 A.
 * v0.3.0 restricts the first qualified operating point to CH1 buck. */

/* GENERIC protocol window, and the OUTER bound: target_vmax_mv() answers
 * min(PD_VMAX_MV, PD_BUCK_VMAX_MV). If this is left behind, the DEVICE SILENTLY CAPS THE
 * DECLARED RANGE - and that is exactly what happened in the 30 V -> 44 V step: PD_BUCK_VMAX_MV
 * moved, this did not, the CAPS reply and the 1 Hz range line (both derived from
 * PD_BUCK_VMAX_MV) advertised ~42 V, and every SET_TARGET above 30 V came back as RANGE with
 * "parameter outside the device's permitted range" while the SAME LOG LINE reported vm=42268.
 *
 * It never showed before because PD_VMAX_MV was always >= PD_BUCK_VMAX_MV (30 V against 18 V,
 * then 20 V, then 24 V) and min() hides the smaller number. Kept equal now, with the assertion
 * below so the pair cannot drift apart again in silence. */
#define PD_VMIN_MV 1000
#define PD_VMAX_MV 44000
/* 5000 mA, raised from 2000 at the operator's request ("limits at least 45 V / 5 A"). The output
 * ceiling is the INPUT POWER and it scales with the LOAD RESISTANCE, so this number decides what
 * output is reachable and the load decides how much of it becomes voltage:
 *
 *   Pin = 20 V * 5.000 A = 100.0 W,  Pout = 98.6 W at the measured 98.6 %
 *   VOUT_max = sqrt(Pout * R)
 *     R = 20 ohm -> 44.4 V    a 24 V target draws 28.8 W: a third of the budget
 *     R = 30 ohm -> 54.4 V
 *     R = 10 ohm -> 31.4 V
 * The 24 V bench point drew 1.184 A in for 29 W out (R = 20 ohm), i.e. 29 % of the old 40 W
 * envelope. THIS IS A PROTECTION LIMIT CHANGE, not a formatting one: the slow CC fold-back now
 * permits ~99 W where it permitted ~40 W.
 *
 * THE CONSEQUENCE THAT HAS TO BE SAID OUT LOUD: 5 A of input is ~2.5x the highest current this
 * stage has ever run. It is not a rating anyone has verified - not the inductor's thermal path,
 * not the FETs' dissipation, not the 2x330 uF input bank's ripple current, and not the supply
 * itself. The 24 V point ran at 40 C. Raising this number does not make the hardware able to
 * deliver 99 W; it only stops the firmware from refusing to try. Bench at 3 A before 5 A. */
#define PD_IMAX_MA 5000
#define PD_DEBUG_ILMAX_MA 500 /* conservative diagnostic request limit, NOT OC */
#define PD_INPUT_NOMINAL_MV 20000
#define PD_VREF_MV 3300 /* CURRENT-channel conversion basis + zero store unit.
                         * The VIN/VOUT channels use PD_VOLTAGE_SCALE_REF_MV. */
#define PD_CALIBRATED 1 /* VIN/VOUT gain+offset fitted from the 5/10/18 V three-point
                         * bench measurement (gain 1000/1000, offset -11 mV); the VIN
                         * channel is still nominal (no VIN pairs were measured) */
#define PD_NTC_VERIFIED 0
#define PD_OUTER_HZ 1000u /* actual CH1 voltage/CC loop rate: 1 kHz task */
#define PD_COMM_TIMEOUT_MS 1000u

/* ============================ CH1 power stage ============================
 * Netlist-verified hardware (功率板.enet, rev 2026-09-03):
 *   Q1/Q3 = input bridge (Q1 high side from 20 V, Q3 low side to GND)
 *   Q2/Q4 = output bridge (Q2 high side to VDCDC_OUT, Q4 low side to GND);
 *   the inductor sits between the two switch nodes through U9 = 1 mOhm, so the
 *   output bridge is a synchronous chopper into the output bank and
 *   VOUT = VIN * d_A / d_B (see the range block below PD_DUTY_MAX).
 *   U8 = 10 uH/25 A (SMMS1770-100M), V_OUT bulk = 3x220 uF/100 V + 4x10 uF
 *   ~= 700 uF. The output capacitors are 100 V parts - an earlier comment in
 *   this file said 25 V, which the netlist (AISHI ERJ1KM221W20OT) disproves.
 *   IOUT shunt R15 = 2 mOhm into INA296A4 (gain 100) = 200 mV/A, zero VDDA/2,
 *   placed between VDCDC_OUT (cap side) and V_OUT (load side), so the output-cap
 *   ripple current does NOT flow through it.
 *   U3/U4 = UCC27301ADR (D/SOIC-8: 1 VDD, 2 HB, 3 HO, 4 HS, 5 HI, 6 LI,
 *   7 VSS, 8 LO), +12 V VDD, 100 nF bootstrap (C10/C15); netlist wires HO to
 *   the high-side gate and LO to the low-side gate on both bridges.
 *   PA3 (CH1_PWR_EN) high enables the four SN74LVC126A buffers that pass
 *   PA8..PA11 to the drivers; PA3 low pulls all four driver inputs low.
 *
 * Switching: master period 21760 counts at 250 kHz (170 MHz, MUL32, 1 count =
 * 183.8 ps). Changed from 500 kHz (10880) in 0.3.21: the dead time is a FIXED
 * 153 ns, so halving the switching frequency halves its share of the period
 * (7.65 % -> 3.83 %) and the commanded minimum duty finally conducts.
 * TA1/TB1 are set at the master period event and reset at CMP1; TA2/TB2 are
 * the dead-time complementary outputs (153 ns, CubeMX fixed).
 *
 * Restricted-buck modulation used here:
 *   input bridge  : CMP1_A = D_A * PER, D_A in [DUTY_MIN, DUTY_MAX = 0.70]
 *   output bridge : CMP1_B = 75 % fixed, so TB2 conducts >= 847 ns per cycle
 *                   and refreshes the output-bridge bootstrap every switching
 *                   period without ever holding both low sides on while the
 *                   input high side is on (CMP1_A + dead time < CMP1_B).
 *   Ideal relation   VOUT = VIN * D_A / 0.75, i.e. the 0.70 ceiling alone would
 *                    cap VOUT at 0.928 * VIN.
 *   Measured relation (0.3.23). The bench measured the effective duty loss twice:
 *   a 20 % command gave 12.35 % at the pin (one 153 ns dead time per high time)
 *   and, in 0.3.22, gate edges of 120 ns turn-on / 55 ns turn-off. The gate-edge
 *   pair does not change the dead time itself but shifts the transfer by the
 *   skew (65 ns), so the pre-0.3.23 model used
 *   d_model = PD_MODEL_DT_COUNTS / PD_SW_PERIOD = 994/21760 = 0.0457:
 *     VOUT = VIN * (D_A - d_model) / (0.75 - d_model)
 *   THE BENCH THEN DISPROVED THAT MODEL: over 204 unsaturated samples the real
 *   transfer is VOUT = 1.1141 mV * count + 1.9 mV at VIN = 19.83 V, i.e. a pure
 *   GAIN deficit of 8.3 % (measured/ideal = 91.7 %) with no offset, so the
 *   achievable output is only 0.856 * VIN. See the PD_RANGE_GAIN_* block below
 *   for the data, the consequences and the still-open question of where the
 *   8.3 % goes.
 *   Register-level timing checks keep using the configured 832 counts (153 ns),
 *   never a model figure: the dead-time generator does not know the gate delays.
 */
#define PD_POWER_STAGE_IMPLEMENTED 1
#define PD_SW_PERIOD 21760u
/* Timer E: ADC scheduling (0.3.21).
 *
 * Timer E drives no output; it only generates the two ADC triggers:
 *   ADC trigger 1 (regular group, ADC1/ADC2, one rank per trigger) = TIMER E CMP3
 *   ADC trigger 2 (injected IL pair, ADC1/ADC2)                    = TIMER E CMP2
 * CubeMX configures exactly that and pd_board.c verifies the registers at boot.
 *
 * Its period MUST stay an integer multiple of the PWM period, otherwise the
 * sample phase walks through the switching cycle and IL/IOUT are sampled at a
 * different point of the ripple from cycle to cycle. At 500 kHz the ratio was
 * 5:1 (54400 = 5 x 10880). 5:1 is impossible at 250 kHz (5 x 21760 = 108800 >
 * 0xFFFF), so the ratio is 2:1 here: 43520 counts = 8 us = 125 kHz, which
 * RAISES the fast IL rate to 125 kHz and the regular sequence rate to 25 kHz.
 * 3:1 (12 us) would be 83.33 kHz - not an integer - and the host validates its
 * DECIMATION against trace_base_hz (1/10/100/1000 must divide it), so 125 kHz
 * is the only clean choice.
 *
 * The trigger placement keeps the sampling instant at 50 % of the PWM period,
 * exactly as the 500 kHz design did:
 *   CMP3 (regular)  = 1/2 PWM period  = 10880 counts = 2.0 us after the rollover
 *   CMP2 (injected) = 3/2 PWM periods = 32640 counts = 6.0 us after the rollover
 * Both instants are 2.0 us (50 %) into their own 4 us PWM cycle.
 *
 * Both triggers must find the ADC IDLE: an injected trigger that arrives while a
 * regular conversion is running is only served after that conversion, which
 * moves the IL sample to a different ripple phase. The budgets below are in
 * HRTIM counts (1 count = 183.8 ps) and are checked at COMPILE time, so a future
 * sample-time or compare-value change cannot silently break the sample phase:
 *   105 ADC cycles (92.5 sample + 12.5 SAR) = 3.706 us = 20161 counts (longest regular rank)
 *    37 ADC cycles (24.5 sample + 12.5 SAR) = 1.306 us =  7104 counts (injected IL)
 *   plus 600 counts (~110 ns) of trigger/start latency allowance.
 * The NTC ranks keep their 92.5-cycle sample time on purpose (untouched in
 * 0.3.21); they are what makes the regular rank the longest one. */
#define PD_TE_PERIOD 43520u  /* 8 us: exactly 2 PWM periods -> 125 kHz */
#define PD_TE_CMP2 32640u    /* injected IL trigger: 3/2 PWM periods = 6.0 us */
#define PD_TE_CMP3 10880u    /* regular-group trigger: 1/2 PWM period = 2.0 us */
/* ADC edge-proximity watch (0.3.32). The two ADC groups are triggered by Timer E at a FIXED
 * phase - 50 % of every PWM period - while the two high-side falling edges move with the
 * duties: the A bridge sweeps A*PD_SW_PERIOD through that instant (A = 50 % lands exactly on
 * it, which is around 11..13.5 V out at 19.8 V in), and the B bridge would too if the B floor
 * were ever opened to 50 %. A sample taken there reads a node that is commutating.
 *
 * This cannot be fixed by moving the trigger (every phase is crossed by some duty) and it must
 * not be "fixed" by rejecting samples, so the firmware's job is to SAY WHEN it happens and let
 * the bench decide whether the reading is affected. Report only - nothing reads these back. */
#define PD_ADC_EDGE_WATCH_COUNTS 816u      /* +/-150 ns: 150 * 5.44 = 816 counts */
#define PD_ADC_EDGE_WATCH_MS 2000u         /* one line per 2 s while the condition holds */
#if PD_TE_CMP3 != PD_SW_PERIOD / 2u
#error "PD_ADC_EDGE_WATCH assumes the regular-group trigger is 50 % into the PWM period"
#endif
#define PD_TE_PWM_RATIO 2u   /* PD_TE_PERIOD == PD_TE_PWM_RATIO * PD_SW_PERIOD */
#define PD_ADC_CYCLE_COUNTS 192u /* one ADC clock (28.33 MHz = 170/6) in HRTIM counts */
#define PD_TE_REG_RANK_COUNTS 20160u /* longest regular rank: 105 ADC cycles = 3.706 us */
#define PD_TE_INJ_RANK_COUNTS 7104u  /* injected IL rank: 37 ADC cycles = 1.306 us */
#define PD_TE_TRIG_LATENCY_COUNTS 600u                     /* ~110 ns allowance */
#define PD_FAST_HZ 125000u /* injected IL pair rate = 1 / PD_TE_PERIOD */
#define PD_TE_GAP_COUNTS(from, to) ((to) > (from) ? (to) - (from) : (to) + PD_TE_PERIOD - (from))
#define PD_TE_GAP_REG_TO_INJ PD_TE_GAP_COUNTS(PD_TE_CMP3, PD_TE_CMP2)
#define PD_TE_GAP_INJ_TO_REG PD_TE_GAP_COUNTS(PD_TE_CMP2, PD_TE_CMP3)

#if PD_TE_REG_RANK_COUNTS != 105u * PD_ADC_CYCLE_COUNTS
#error "PD_TE_REG_RANK_COUNTS must stay 105 ADC cycles (92.5 sample + 12.5 SAR)"
#endif
#if PD_TE_INJ_RANK_COUNTS != 37u * PD_ADC_CYCLE_COUNTS
#error "PD_TE_INJ_RANK_COUNTS must stay 37 ADC cycles (24.5 sample + 12.5 SAR)"
#endif
#if PD_TE_PERIOD != PD_TE_PWM_RATIO * PD_SW_PERIOD
#error "PD_TE_PERIOD must be an integer number of PWM periods (sample phase lock)"
#endif
#if PD_TE_CMP3 != PD_SW_PERIOD / 2u
#error "the regular-group trigger must stay at 50 % of a PWM period"
#endif
#if PD_TE_CMP2 != PD_TE_PERIOD - PD_SW_PERIOD / 2u
#error "the injected IL trigger must stay at 50 % of a PWM period"
#endif
#if PD_TE_CMP2 >= PD_TE_PERIOD || PD_TE_CMP3 >= PD_TE_PERIOD || PD_TE_CMP2 == PD_TE_CMP3
#error "Timer E compare values must be distinct and below its period"
#endif
#if PD_TE_GAP_REG_TO_INJ < (PD_TE_REG_RANK_COUNTS + PD_TE_TRIG_LATENCY_COUNTS)
#error "the injected IL trigger can arrive while a regular conversion still runs"
#endif
#if PD_TE_GAP_INJ_TO_REG < (PD_TE_INJ_RANK_COUNTS + PD_TE_TRIG_LATENCY_COUNTS)
#error "the regular trigger can arrive while the injected conversion still runs"
#endif
/* HRTIM high-resolution (MUL32) minimum compare value. ST's own driver header
   (Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_ll_hrtim.h, LL_HRTIM_TIM_SetCompare1)
   states: "Compare value must be above or equal to 3 periods of the fHRTIM
   clock, that is 0x60 if CKPSC[2:0] = 0". This project uses
   HRTIM_PRESCALERRATIO_MUL32 = 0, i.e. CKPSC = 0, so 0x60 = 96 is the floor.
   Values below it are outside the documented range. The actual waveform and
   the cause of the bench over-current remain unverified. Never write one. */
#define PD_CMP_MIN 96u
/* Compare-commit window (0.3.19).
 *
 * CMP1xR of TIMER A and TIMER B are preloaded; both are transferred to the
 * active registers by the MASTER timer update event (verified in the .ioc and
 * in Core/Src/hrtim.c: PreloadEnable=ENABLED, UpdateTrigger1=MASTER,
 * NumberUpdateTrigger=1, ResetUpdate=DISABLED). The pair is only coherent if
 * both writes complete before THAT SAME update event, which occurs when the
 * master counter reaches PD_SW_PERIOD. Masking interrupts does not stop the
 * counter, so the guarantee has to be a time margin, not the critical section.
 *
 * 1 core cycle (170 MHz) = 32 HRTIM high-resolution counts (183.8 ps each).
 * PD_CMP_COMMIT_MARGIN = 3000 counts = 552 ns = 94 core cycles, i.e. roughly
 * 2.5x the masked section (PD_PortSetCompare measures it as crit_cycles; the
 * 1 Hz log prints the worst case so the margin can be checked on hardware).
 * PD_CMP_LATEST is therefore the newest counter value at which a commit may
 * still start: at 18760 (250 kHz) the remaining 3000 counts to the update event
 * are more than the whole masked section needs. The margin is a TIME budget, so
 * it does not scale with the period: 552 ns is 13.8 % of the 4 us period (was
 * 27.6 % at 500 kHz), and a full retry sequence costs at most
 * PD_CMP_MAX_RETRIES x 4 us = 128 us, far inside the 1 ms control tick.
 *
 * The old form waited for "just after the rollover" (MCNTR < 1500 = 276 ns)
 * and then refused the commit if the window had closed between the unlocked
 * check and the masked re-check - a few percent of all attempts, which the
 * 1 kHz loop reported as PD_FAULTS_INTERNAL. Retrying on the next period is
 * safe because nothing was written; a real timeout still stops the stage. */
#define PD_CMP_COMMIT_MARGIN 3000u
#define PD_CMP_LATEST (PD_SW_PERIOD - PD_CMP_COMMIT_MARGIN)
#define PD_CMP_MAX_RETRIES 32u
#define PD_CMP_WAIT_GUARD 20000u
#define PD_OUT_BRIDGE_CMP 16320u /* TB1 75 % of the 4 us period -> 847 ns TB2 window */
#define PD_OUT_BRIDGE_DUTY 0.75f
#define PD_DUTY_MIN 0.05f
/* ===================== CH1 restricted-boost-capable range (0.3.22) ========
 * The output bridge is a synchronous chopper to the output bank, so the
 * conversion ratio of this four-switch stage is
 *
 *     VOUT = VIN * d_A / d_B          (d_B = PD_OUT_BRIDGE_DUTY, fixed at 75 %)
 *
 * which is why the stage is a RESTRICTED buck while d_A <= d_B: with the
 * ceilings below the reachable output is VOUT <= VIN * (A_MAX-Ddt)/(B-Ddt)
 * = 0.928 * VIN, i.e. 18.6 V at 20 V in and 21.3 V at 23 V in. THIS version
 * implements 2..18 V (the bench-verified target); the OVERALL project target is
 * 2..24 V, and 24 V is NOT reachable inside the 17..23 V input window with
 * d_B held at 75 % - it needs either VIN >= ~25.9 V or a scheduled d_B
 * (VOUT/VIN = d_A/d_B, i.e. the boost-ish regime the current milestone plan
 * deliberately excludes because it changes the loop gain, the feed-forward and
 * the A/B edge analysis). Raising PD_BUCK_VMAX_MV alone is therefore NOT
 * enough for 24 V; host test test_declared_output_range_is_reachable_inside_the_
 * input_window() fails if the declared range and the input window disagree.
 */
#define PD_DUTY_MAX 0.70f
/* Integer boundary for the SAME ceiling. Every consumer of the duty ceiling
 * (feed-forward clamp, integral anti-windup, final compare value, regulation-
 * failure test) uses these counts, not a rounded float, so no site can end up
 * one count above another. */
#define PD_CMP_A_MAX 15232u /* 70.00 % of PD_SW_PERIOD */
/* ============ dual-bridge distributor (0.3.28, plan phase A) ==================
 * PD_MOD_LINKED selects which modulator the voltage loop drives:
 *   0 = legacy: A regulated, B FIXED at 75 %. This is the regression baseline of
 *       0.3.27 and stays compilable on purpose (plan section 12: 旧版作回退基线).
 *   1 = linked: one control quantity q is split between both bridges, and the
 *       distributor (pd_modulation.c) enforces the ratio DA/DB = q/0.75.
 * The five bound families below are deliberately separate (plan section 3.2):
 *   q range      - what the released EXPERIMENT may ask for;
 *   A bounds     - the physical A-bridge ceiling and the minimum pulse;
 *   B bounds     - the experimental B floor of this release stage and the B ceiling;
 *   integral     - the controller's own integrator range;
 *   commit       - the register-level checks in pd_power.c.
 * Sharing one constant between them is what the plan forbids.
 *
 * Overridable on the command line (-DPD_MOD_LINKED=0) so the fallback build is COMPILED by the
 * release gate instead of being dead code that only looks maintained. tools/compile_variants.py
 * builds that variant with the same -Werror flags as production. */
#ifndef PD_MOD_LINKED
#define PD_MOD_LINKED 1
#endif
/* Where the B floor stands now: 30 %, well past the 50 % "phase C" the plan stopped at, because
 * the operator asked for it and the bench has run there since 0.3.37. q_max = 1.75 is the SAME
 * constraint seen from the other side of q = 0.75*A/B, and pd_mod_pair_check keeps the two in step.
 * Lowering this further is the one lever that buys output range: the ceiling is A at 70 %, so
 * VOUT/VIN ~ k_v*(A_max-dt)/(B_min-dt), and k_v itself falls as the ratio rises - see the measured
 * numbers in the PD_RANGE_GAIN_LINKED_NUM comment. */
#define PD_MOD_B_MIN_DUTY 0.2757f
#define PD_MOD_B_MIN_COUNTS 6000u  /* 27.57 % of PD_SW_PERIOD: STEP 1 of the 44 V plan (0.3.48).
                                    * Was 6528 (30 %). The 2026-09-19 bench fit says 44 V over the
                                    * whole 19..21 V input window would need B <= 5804 (26.67 %), and
                                    * that is an 8.6 % extrapolation of the fit, so the floor comes
                                    * down in two steps and this is the first: enough for 44 V at a
                                    * 20 V input with margin, and enough to MEASURE whether the fit
                                    * holds at 42/43 V before going further.
                                    * Self-check of that fit: it predicts the old saturation ratio
                                    * as 2.0642 against 2.0647 measured - 0.02 %.
                                    * Bootstrap: 6000 counts = 1103 ns of low side per 4 us period,
                                    * still well above any refresh requirement, but this is the hard
                                    * constraint to confirm before the second step to 5700.
                                    * A pinned at PD_CMP_A_MAX
                                    * with B on the floor is the end of the actuator:
                                    *   law  = (A_max - dt) / (B_min - dt) = 14238 / 5534 = 2.5728
                                    * This block used to claim an effective gain of 0.907 * that,
                                    * i.e. 2.334 V/V and 46.7 V from a 20 V input. THE BENCH REFUTED
                                    * IT on 2026-09-19: at 19.81 V input with B on the floor the output
                                    * saturated at 40899 mV, an effective k_v of 0.8025 and a ratio of
                                    * 2.0647. The old number was 13 % high because k_v at the 24 V
                                    * point (0.907) does not hold at a 2x step-up. Load was 0.5 A, so
                                    * it was the MODULATOR that ran out, not the input current -
                                    * PD_IMAX_MA never became the binding constraint. */
#define PD_MOD_Q_MAX_DUTY 1.904f
#define PD_MOD_Q_MAX_COUNTS 41431u /* 190.4 % of PD_SW_PERIOD: B floor reached. On the SAME identity,
                                    * to within one period count:
                                    *   q_max * B_min = 41431 * 6000 = 248,586,000
                                    *   0.75 * A_max * P = 0.75 * 15232 * 21760 = 248,586,240
                                    * difference 240 <= PD_SW_PERIOD, so pd_mod_pair_check (which
                                    * allows +/- one period) and pd_mod_range_check both hold.
                                    * 38080/6528 was the 30 % form; this is the 6000-count step 1.
                                    * Above 100 % on
                                    * purpose: q is the equivalent A duty, so a q above the period
                                    * means "A at its ceiling and B below 75 %", not a duty the A
                                    * bridge could ever produce (the distributor pins A at
                                    * PD_CMP_A_MAX from q = 0.75 upwards). */    
/* Stage C step 2 (0.3.34): the operator asked for a 50 % B floor, which is the point where the
 * output bridge's falling edge (B*PD_SW_PERIOD) lands EXACTLY on the ADC sampling instant
 * (PD_TE_CMP3 = PD_SW_PERIOD/2 = 2000 ns). Unlike a sweep through that phase, this is a STEADY
 * condition: whenever the loop sits on the floor, every VIN/VOUT/IOUT/IL sample is taken on a
 * node that is commutating, and because 250 kHz/25 = 10 the error aliases to a fixed offset
 * rather than to noise.
 *
 * It is opened deliberately and it is OBSERVABLE: PD_PowerAdcEdgeNear() reports it as code 15,
 * so the bench decides whether the reading actually suffers. If it does, the options in order of
 * cost are (1) keep the operating point off the exact floor, (2) move the sampling phase (a
 * CubeMX change with its own verification chain). */
/* Stage C step 1 (0.3.33). Why 55 % and not lower: the ADC samples every switching period at
 * PD_TE_CMP3 = PD_SW_PERIOD/2 = 2000 ns, and the output bridge's falling edge sits at
 * B*PD_SW_PERIOD - so B = 50 % puts that edge exactly on the sampling instant and B = 54 %
 * leaves only 160 ns for a commutation that rings for about 120 ns. 55 % keeps 200 ns, which is
 * the working floor. Going below it means moving the sampling phase (a CubeMX change with its
 * own verification chain), which is explicitly NOT done here.
 *
 * The plan's condition for opening this step was "solve the ADC edge proximity first". It is now
 * MEASURED rather than argued: PD_PowerAdcEdgeNear() (0.3.32) reports when either falling edge
 * enters the window, so the bench can see it in the log (code 15) instead of trusting a
 * prediction. Any target that lights code 15 is a target whose telemetry is suspect. */
#define PD_MOD_Q_MIN_DUTY 0.05f
#define PD_MOD_Q_MIN_COUNTS 1088u  /* 5.00 %: same floor the A bridge already had */
/* Zone boundaries of the distribution function, as smoothstep parameters:
 * DA = 0.65 + 0.10*t - 0.05*t^2 over q in [0.65, 0.75], so DA reaches the 70 % ceiling
 * exactly at q = 0.75 with zero slope, and DA(0.65) = 0.65 with unit slope. */
#define PD_MOD_Q_LOW_DUTY 0.65f
#define PD_MOD_Q_LOW_COUNTS 14144u  /* 65.00 %: end of the fixed-B region */
#define PD_MOD_Q_HIGH_DUTY 0.75f
#define PD_MOD_Q_HIGH_COUNTS 16320u /* 75.00 %: start of the DA = 70 % region */
/* Minimum non-zero A pulse: the old PD_DUTY_MIN, now expressed where the distributor can use
 * it. It stays a NON-ZERO floor only; a zero request goes through the power state machine. */
#define PD_MOD_A_MIN_COUNTS 1088u
/* The A floor must not sit ABOVE the q floor. In the low zone the distributor hands A = q, so if
 * A_min > q_min the A clamp would fire at q = q_min and push the committed pair to a q LARGER than
 * the one requested - a floor that silently commands more than the control law asked for, exactly
 * the failure the anti-windup would then have to fight. Equal today; the assertion keeps a future
 * retune from breaking it quietly. */
typedef char pd_mod_amin_check[(PD_MOD_A_MIN_COUNTS <= PD_MOD_Q_MIN_COUNTS) ? 1 : -1];
/* Compile-time proof that the released q range is consistent with the B floor: at q_max the
 * distributor must not need a B below the experimental floor. From DA = 0.70 (high zone) and
 * DB = 0.525/q: q_max * B_min <= 0.525 in duty units, i.e. in counts q_max * B_min <= 1.5 *
 * period. A violation fails the build rather than silently clamping B. */
/* Compile-time assertion (not #if: the product needs 64-bit arithmetic, which the preprocessor
 * cannot do). Written as a positive assertion so the compiler reports the reason. */
typedef char pd_mod_range_check
    [(PD_MOD_Q_MAX_COUNTS * PD_MOD_B_MIN_COUNTS * 2) <=
             (PD_SW_PERIOD * PD_SW_PERIOD * 3) ? 1 : -1];
/* PD_CTRL_LIMIT_CNT in pd_power.c uses PD_MOD_Q_MAX_COUNTS as "the end of the actuator's
 * range", and the duty form PD_MOD_Q_MAX_DUTY is the controller's clamp. Those two must be the
 * same number, or the regulation-failure backstop and the control clamp would disagree about
 * where the actuator stops. 0.875 = 7/8 exactly, so this is an equality, not a rounding. */
/* The q ceiling and the B floor are the SAME constraint seen from the two sides of
 * q = 0.75*A/B, so they must agree to within one period count: a build where they disagree
 * would let the distributor ask for a pair that commit_counts() then has to clamp, which is
 * exactly the "the actuator did not follow" case the anti-windup exists to avoid. */
typedef char pd_mod_pair_check[
    (PD_MOD_Q_MAX_COUNTS * PD_MOD_B_MIN_COUNTS) + PD_SW_PERIOD >=
        (3LL * PD_CMP_A_MAX * PD_SW_PERIOD / 4) &&
    (PD_MOD_Q_MAX_COUNTS * PD_MOD_B_MIN_COUNTS) <=
        (3LL * PD_CMP_A_MAX * PD_SW_PERIOD / 4) + PD_SW_PERIOD ? 1 : -1];
#if PD_MOD_Q_LOW_COUNTS >= PD_MOD_Q_HIGH_COUNTS
#error "the transition zone must have positive width"
#endif
/* The transition polynomial DA = 0.65 + 0.10t - 0.05t^2 uses span = 0.10 of the period as its
 * normaliser, so the zone width MUST be that value. Writing it twice is how the arithmetic went
 * wrong in review; asserting it here makes the identity impossible to break silently. */
typedef char pd_mod_span_check
    [(PD_MOD_Q_HIGH_COUNTS - PD_MOD_Q_LOW_COUNTS) * 10 == PD_SW_PERIOD ? 1 : -1];
#if PD_MOD_B_MIN_COUNTS >= PD_OUT_BRIDGE_CMP
#error "the experimental B floor must sit below the fixed B ceiling"
#endif
#if PD_MOD_LINKED && (PD_MOD_Q_MAX_COUNTS <= PD_MOD_Q_HIGH_COUNTS)
#error "linked mode with q_max at or below the high-zone entry cannot reach the B floor"
#endif
/* One configured dead time in HRTIM high-resolution counts: 20 fHRTIM clocks at
 * DIV1 = 117.6 ns = 640 counts (32 counts per core cycle). Used for the
 * REGISTER-LEVEL edge-separation checks below.
 *
 * 0.3.30 reduced this from 26 clocks (152.9 ns). Why: the 2026-09-13 scope session
 * measured the transfer deficit as the time per period during which the inductor is
 * connected to VOUT through body diodes - 260 ns, i.e. 6.5 % of the period - and that
 * deficit is what forced 18 V out of a 19.8 V input to sit with A pinned on its 70 %
 * ceiling, both bridges commutating together. The dead time is the part of that window
 * the FIRMWARE owns (the other part is the gate drive's turn-on/turn-off asymmetry).
 * Shortening it was cleared by the measured interlock: "Q2 Vgs below 2 V -> Q4 gate above
 * 2 V" was 92..100 ns against a 73 ns requirement, so the 35 ns this removes still leaves
 * it above the 40 ns floor. RE-MEASURE that interlock after any change here: the gate
 * level dead time is longer than the configured one (167 ns measured against 153 ns
 * configured at the time), so the margin is not the number configured. */
#define PD_CMP_DT_COUNTS 640u
/* Measured gate-edge delays (bench, CH1, 0.3.22): turn-on 120 ns, turn-off
 * 55 ns. They are NOT part of the register model: the dead-time generator knows
 * nothing about them, so any statement about which device actually CONDUCTS
 * must use the effective skew (120-55) = 65 ns = 354 counts. Used only for the
 * regulation model (feed-forward, available-output-range), never for the
 * register timing gates. */
#define PD_EDGE_ON_NS 120u
#define PD_EDGE_OFF_NS 55u
/* 1 ns = 5.44 HRTIM counts (5.44 GHz); rounded to nearest. Keep the two macros
 * below as plain literals so tooling (tools/check_configuration.py, the host
 * suite) can read the numbers textually; the #if gates prove they still equal
 * the expressions. */
#define PD_NS_TO_COUNTS(ns) ((((ns) * 544u) + 50u) / 100u)
#define PD_MODEL_SKEW_COUNTS 354u  /* (120-55) ns = 65 ns -> 354 counts */
#define PD_MODEL_DT_COUNTS 994u    /* 640 + 354: what the duty->voltage transfer loses */
#if PD_MODEL_SKEW_COUNTS != PD_NS_TO_COUNTS(PD_EDGE_ON_NS - PD_EDGE_OFF_NS)
#error "PD_MODEL_SKEW_COUNTS does not match the measured gate edges"
#endif
#if PD_MODEL_DT_COUNTS != (PD_CMP_DT_COUNTS + PD_MODEL_SKEW_COUNTS)
#error "PD_MODEL_DT_COUNTS must be the configured dead time plus the measured skew"
#endif
/* A-bridge ceiling where the MIDDLE interval still conducts through channels:
 *   (B + tau_off) - (A + DTR + tau_on) > 0
 * (tau in counts: 120 ns = 653, 55 ns = 299).
 * With the 0.3.30 dead time (640 counts) the derivation gives
 *   B + 299 - 640 - 653 = 15326, i.e. ABOVE PD_CMP_A_MAX (15232): inside the allowed A
 * range the middle interval now always conducts through the low-side channels, and the
 * boundary is no longer reachable. The constant below is deliberately KEPT at the value
 * derived from the OLD 832-count dead time: it is a conservative (lower) boundary, it is
 * asserted to sit below the A ceiling, and a boundary that fires early can only under-
 * state the margin, never hide a problem. Do not raise it without re-deriving. */
#define PD_CMP_MID_CHANNEL_MAX 15134u
/* Hard structural limit: A must stay below B or the stage leaves the restricted
 * buck regime (the interval [B, A] would put VIN straight across the inductor).
 * The measured-edge overlap of Q1 and Q4 would only occur above B + DTF + skew
 * = 17506 counts (80.4 %), but that regime is not this milestone. */
#if PD_CMP_A_MAX >= PD_OUT_BRIDGE_CMP
#error "A-bridge ceiling must stay below the fixed output-bridge duty (restricted buck)"
#endif
#if PD_CMP_A_MAX + PD_CMP_DT_COUNTS >= PD_OUT_BRIDGE_CMP
#error "A-bridge ceiling + dead time must stay below the output-bridge duty"
#endif
#if PD_CMP_MID_CHANNEL_MAX >= PD_CMP_A_MAX
#error "the middle-interval boundary must be below the A-bridge ceiling"
#endif
#define PD_BUCK_VMIN_MV 2000
/* 0.3.31 raised the declared maximum from 18000 to 20000 mV so the boost region
 * (VOUT > VIN) can be demonstrated with the supply the bench actually has (19..20 V).
 * The stages below this one stay at 18 V; the new top is an experiment, not a new
 * production claim, and README/VALIDATION say so.
 *
 * PROTECTION CONSEQUENCE, read before touching this number: PD_OVP_ABS_MAX_MV is DERIVED
 * from it (x1.15), so 20 V declared means a 23 V OVP threshold. That is permissible only
 * because the power components are far above it - taken from the power-board netlist:
 *   U5/U6/U7  220 uF ERJ1KM221W20OT   rated 100 V
 *   C25-C28   10 uF FS32X106K101EGG  rated 100 V (1210 X7R)
 *   Q1-Q4     CSD19531Q5A             Vdss 100 V
 * 23 V is 23 % of those ratings. PD_OUT_CAP_RATED_MV below records the number and
 * tools/check_configuration.py refuses a build whose OVP ceiling exceeds 85 % of it, so a
 * future range change cannot quietly outrun the hardware. */
/* Declared maximum output: 44 V. The operator asked for limits AT LEAST 45 V / 5 A, and the two
 * halves of that resolve very differently - both are recorded here rather than smoothed over.
 *
 * THE DUTY CEILING IS NO LONGER THE CONSTRAINT. With B on the 30 % floor and A pinned at
 * PD_CMP_A_MAX the effective transfer is 0.907 * (15232-994)/(6528-994) = 2.334 V/V, so the
 * topology allows VIN * (q_max-dt)/(B_ceiling-dt) = 21000 * 37086/15326 = 50.8 V. From a 21 V
 * input the TOPOLOGY can therefore reach well past 45 V.
 *
 * WHAT ACTUALLY STOPS IT IS THE RANGE MODEL, and that is a measurement question, not a
 * configuration one. PD_PowerAllowedVmaxMv() answers with the MEASURED transfer line
 * (PD_RANGE_GAIN_*). At the 21 V input ceiling that model now gives
 *   21000 * 2060/1000 - 200 = 43060 mV
 * and check_configuration.py rejects any declared value above it. 39000 sits below that bound with
 * margin, so the declaration is one the firmware can keep.
 *
 * WHY IT CAME DOWN FROM 44000 (2026-09-19 bench, log 28): 44000 was justified by the extrapolated
 * gain 2140, which turned out 3.6 % optimistic. At 19.81 V input a 42000 mV setpoint was ACCEPTED by
 * the gate and then could not be reached - the stage stopped on REGULATION FAILED with A pinned at
 * 70 % and B on its 30 % floor, having reached 40899 mV. The re-fitted gain is 2060, so the same
 * setpoint is now refused at SET_TARGET instead of being attempted and faulting.
 *
 * THE WINDOW'S LOW END IS TIGHTER THAN ITS HIGH END. The number above uses 21 V; at the 19 V floor
 * of the window the same line gives 19000 * 2060/1000 - 200 = 38940 mV, i.e. 60 mV BELOW the 39000
 * declared here. So at the very bottom of the input window the top 60 mV of the declared range is
 * refused - correct behaviour, but it is why the release gate prints the vin_min figure too. Declare
 * 38900 instead if the range must hold across the whole window.
 *
 * WHAT 44 V WOULD NEED, NOW MEASURED RATHER THAN GUESSED: a ratio of 2.20 at 20 V input, against the
 * 2.0647 the hardware actually delivered. The A ceiling is fixed by the bootstrap high side, so the
 * only software lever is the B floor: 2.20 / 0.8025 = 2.741 = (A_max-dt)/(B_min-dt) needs B ~ 6189
 * counts, i.e. a ~28.4 % floor with q_max ~ 1.85. That is a real change to the switching stress and
 * the dead-time fraction, and k_v falls further as the ratio rises, so the last volt is the most
 * expensive one. Do not simply type a bigger number in.
 *
 * 5 A OF INPUT CURRENT IS A DIFFERENT STORY AND IS SET BELOW. The output ceiling is input POWER:
 *   Pin = VIN * IIN, Pout = Pin * 0.986, VOUT_max = sqrt(Pout * R)
 *   VIN 20 V, IIN 5 A -> 98.6 W -> 20 ohm: 44.4 V   30 ohm: 54.4 V   10 ohm: 31.4 V
 *   VIN 21 V, IIN 5 A -> 103.5 W -> 20 ohm: 45.5 V  30 ohm: 55.7 V
 * so 45 V into 20 ohm needs 5.13 A of input, and 45 V into 30 ohm needs 3.42 A. 45 V AT 5 A OF
 * OUTPUT (225 W) would need ~11.4 A of input and is not reachable at any setting. */
#define PD_BUCK_VMAX_MV 44000
/* The declared CH1 range must fit inside the generic window: target_vmax_mv() takes the SMALLER
 * of the two, so a violation does not fail loudly at runtime, it silently narrows the range. */
typedef char pd_vmax_pair_check[(PD_BUCK_VMAX_MV <= PD_VMAX_MV) ? 1 : -1];
typedef char pd_vmin_pair_check[(PD_BUCK_VMIN_MV >= PD_VMIN_MV) ? 1 : -1];
/* Rated voltage of the components that sit across VOUT (netlist above). Used ONLY by the
 * configuration gate that bounds PD_OVP_ABS_MAX_MV; no runtime code reads it. */
#define PD_OUT_CAP_RATED_MV 100000
/* Available-output-range model (0.3.23, MEASURED).
 *
 * 0.3.22 used the topology/edge model
 *   model_vmax = floor(VIN*(A_MAX - REG_RESERVE - Ddt)/(B - Ddt)) - LOSS
 * with Ddt = PD_MODEL_DT_COUNTS (994 counts since 0.3.30). The bench then showed that model
 * OVERPREDICTS the real stage by about 8 %: at VIN = 19.83 V the loop saturated
 * A at the 15232 ceiling and VOUT stopped at 16.96 V instead of the predicted
 * 18.40 V, which made 17 V run silently saturated and made 18 V trip the
 * regulation-failure backstop after 500 ms (PD_FAULTS_INTERNAL, logged
 * "REGULATION FAILED: duty saturated, VOUT below target").
 *
 * Measurement (PowerDesk_测量记录.csv, 204 unsaturated samples over 1.7..16 V,
 * VIN = 19.83 V): VOUT = 1.1141 mV * duty + 1.9 mV, i.e. a pure GAIN deficit with
 * essentially zero offset - the real gain is 91.7 % of the ideal VIN/B, and
 * VOUT_max = 0.856 * VIN. The 18 V point measured afterwards agrees within 0.5 %
 * (VIN 21.0 V -> 17.891 V regulation, DMM 17.82 V).
 *
 * So the available range is the MEASURED line below, not the topology model:
 *   allowed_vmax = min(PD_BUCK_VMAX_MV, floor(VIN*856/1000) - 200)
 * The 200 mV margin covers the fit's residual (~0.5 %) and leaves the loop room
 * to regulate instead of sitting on the ceiling. The topology model is kept as a
 * documented UPPER BOUND only: the host suite asserts the measured line never
 * exceeds it, so a bad fit can never make the firmware promise more than the
 * duty ceiling can physically deliver.
 *
 * Required input voltage for a given target (0.856, no margin / with 200 mV):
 *   16 V: 18.69 / 18.93   17 V: 19.86 / 20.09   18 V: 21.03 / 21.26
 *   19 V: 22.20 / 22.43   20 V: 23.36 / 23.60 (outside the 17..23 V window)
 * At the top of the input window (23 V) the stage reaches about 19.7 V.
 *
 * WHY 8.3 % IS LOST IS STILL OPEN: solving the volt-second balance with the
 * measured VOUT means either the output-bridge high side conducts ~95 ns longer
 * per period than commanded, or the input-bridge high side loses ~234 ns more
 * than the configured 153 ns dead time (both are scope-level claims, NOT
 * measured). If that is a drive/bootstrap issue, fixing it would raise the whole
 * range (and help the 24 V milestone), so PD_RANGE_GAIN_* must be RE-MEASURED
 * after any change to the gate drive, dead time or output-bridge duty. Do not
 * silently adjust it to make a target fit. */
/* The bench record behind the ratio above, kept here so tools/apply_voltage_calibration.py
 * can compute how much a calibration would move the line (it needs the duty CEILING, not
 * the full period, because that is where the ratio was measured):
 *   firmware VOUT = slope * duty_count + intercept, 204 unsaturated points, VIN = 19827 mV. */
#define PD_RANGE_FIT_SLOPE_MV_PER_COUNT 1.1141f
#define PD_RANGE_FIT_INTERCEPT_MV 1.9f
#define PD_RANGE_FIT_VIN_MV 19827
#define PD_RANGE_GAIN_NUM 856u  /* measured VOUT_max/VIN with A ALONE, ceiling 0.70 */
#define PD_RANGE_GAIN_DEN 1000u
/* ---- the same fit re-expressed in the LINKED coordinate (0.3.28) -----------------------
 * 856 carries the A-bridge ceiling inside it: 0.856 = k_v * 0.70/0.75 with k_v = 0.917, and
 * 0.917 is exactly the "91.7 % of ideal" the 204-point fit produced. In the linked
 * coordinate the only change is WHERE the ceiling is: q_max = 0.875 instead of A = 0.70, in
 * the same transfer relation VOUT = k_v * VIN * (x/0.75). So the same measurement predicts
 *     VOUT_max = k_v * q_max/0.75 * VIN = 0.916 * 1.16667 * VIN = 1.068 * VIN.
 * k_v = 0.916 (the 16 V point) is used, not 0.9307 (the 5 V one): the two measured points do
 * not agree, and the lower one is the honest input to a model that decides whether a target
 * is ACCEPTED. The dead-time-aware form of the same topology gives 1.084, i.e. 1.5 % more -
 * the smaller number is taken on purpose.
 *
 * THIS IS AN EXTRAPOLATION, NOT A MEASURED POINT: every bench sample so far had B fixed at
 * 75 %, and the two points that define k_v do not bound what happens when BOTH bridges sit
 * near 70 % - which is exactly where "18 V from a 20 V supply" lands (A 69.97 %, B 70.74 % at
 * VIN = 19.83 V). Phase B of 实现方案_CH1双桥联动调制与分阶段升压.md (15 -> 18 -> near-VIN ->
 * slight boost) must confirm this number or replace it with a re-fitted line. Until then the
 * 200 mV margin, the structural upper bound and PD_RANGE_MEASURED_CAL all stay in force, and
 * PD_MOD_LINKED = 0 keeps using 856 - a build that promises the linked range while running
 * the old modulator is precisely the 0.3.22 failure. */
/* MEASURED at the stage-C ceiling (2026-09-19). The 1068 that shipped from 0.3.28 was the line
 * at q_max = 0.875; the one below is fitted where the stage now works. Two independent data sets
 * agree and both show k_v falling monotonically with the duty:
 *   VIN 19.8 V: 0.9339 (10 V out) .. 0.9262 (20 V)
 *   VIN 21.0 V: 0.9167 (21 V) 0.9119 (22 V) 0.9081 (23 V) 0.9042 (24 V)
 * so the gain is k_v at the CEILING: 0.9042 x 0.9545/0.75 = 1151. With it, 24 V needs
 *   VIN >= 24000 * 1.15 / 1.151 = 21026 mV
 * which is exactly what the bench had to supply (21 V, at B = 55.38 %, lim = 0x00).
 *
 * A SINGLE k_v IS STILL AN APPROXIMATION, and 2026-09-19 measured how much: the effective k_v falls
 * monotonically with the step-up ratio, so a straight line through the 24 V point is optimistic at
 * the top of the range. Making it duty-dependent remains the natural next step and is still safe -
 * it changes only which targets are ACCEPTED, never the modulation or the protections. */
/* RE-FITTED FROM THE 2026-09-19 BENCH RUN (log 28 + measurement record 14). The comment this
 * replaces said "k_v falls with duty, so 28 V and 30 V points must be re-measured before those
 * numbers are treated as fitted" - and that is exactly what happened. With A pinned at its 70 %
 * ceiling and B modulating, measured on the bench:
 *   VOUT 38005 mV, VIN 19827, B = 7085  ->  k_v 0.8201   (steady, held 6 s)
 *   VOUT 40899 mV, VIN 19809, B = 6528  ->  k_v 0.8025   (B on its floor: end of the actuator)
 * against 0.907 at the 24 V / B = 51.3 % point.
 * The maximum ratio actually delivered at 19.81 V input was 40899/19809 = 2.0647, so this value is
 * 2065 rounded DOWN to 2060 for margin. The old 2140 came from 856 * 38080/15232, the scaled legacy
 * fit, and was 3.6 % optimistic: it made PD_PowerAllowedVmaxMv() promise 42210 mV where the stage
 * saturated at 40899 mV, so a 42000 mV setpoint was accepted and then stopped on REGULATION FAILED.
 * The gate's band for this value was [scaled*0.9, scaled+4] = [1926, 2144] at q_max = 38080; 2060
 * sat inside it. */
/* RE-PREDICTED FOR THE 6000-COUNT B FLOOR (step 1 of the 44 V plan). Raising the q ceiling raises the
 * law ratio at the floor, so the predicted maximum ratio rises with it - and the prediction now comes
 * from the bench-fit k_v curve rather than from the gate's scaling law:
 *   measured   k_v = 0.9669 - 0.0797 * (VOUT/VIN)      residual <= 0.53 % over ratio 1.52..2.06
 *   law at floor (A_max-dt)/(B_min-dt) = 14238/5006 = 2.84419
 *   solve r = k_v(r) * 2.84419  ->  r = 2.24207  ->  2242
 * The gate's scaling law would say 856 * 41431/15232 = 2328, because it ASSUMES k_v is constant and
 * the ratio scales with the ceiling. The 2026-09-19 measurements disproved that assumption - k_v falls
 * from 0.8456 at ratio 1.52 to 0.8025 at 2.06 - so 2242 is deliberately 3.7 % MORE CONSERVATIVE than
 * the scaling law. The gate's band is now [2095, 2332] and 2242 sits inside it.
 * STILL A PREDICTION: the fit covers ratio 1.52..2.06 and 2.242 is +8.6 % beyond it. Re-measure at
 * 42/43 V and replace this before 44 V is declared. */
#define PD_RANGE_GAIN_LINKED_NUM 2242u
#if PD_MOD_LINKED
#define PD_RANGE_GAIN_ACTIVE_NUM 2242u /* = PD_RANGE_GAIN_LINKED_NUM */
#else
#define PD_RANGE_GAIN_ACTIVE_NUM 856u  /* = PD_RANGE_GAIN_NUM */
#endif
/* COUPLING WITH THE VOUT CALIBRATION - read before touching either one.
 * PD_RANGE_GAIN_NUM/DEN was fitted from the firmware's UNCALIBRATED VOUT telemetry
 * (the 204-sample bench record). Once PD_CALIBRATED = 1, the VOUT the firmware
 * reports - and therefore the transfer line this coefficient describes - shifts by
 * the calibration factor, so keeping the old ratio would apply the correction TWICE:
 * a silent 1-2 % range error, i.e. 180-360 mV at 18 V, enough to refuse a usable
 * target or to promise one the stage cannot hold.
 * PD_RANGE_MEASURED_CAL records that the transfer line was re-measured AFTER the
 * calibration was applied: run the VOUT-vs-duty analysis again with the calibrated
 * firmware (unsaturated points), put the new ratio above, then set this to 1.
 * pd_power.c refuses to compile PD_CALIBRATED with PD_RANGE_MEASURED_CAL = 0,
 * tools/check_configuration.py reports it while the configuration is being edited,
 * and tools/apply_voltage_calibration.py refuses to write a calibration without
 * --range-remeasured. The sequence cannot be skipped by accident. */
/* Provenance of the ratio above, three states so the value cannot lie:
 *   0  fitted from UNCALIBRATED VOUT telemetry (cannot coexist with PD_CALIBRATED = 1)
 *   1  RE-MEASURED with the calibrated build (a fresh VOUT-vs-duty log session)
 *   2  RECOMPUTED from the fitting record through the calibration; the effect was
 *      quantified and is negligible (see tools/apply_voltage_calibration.py output)
 * This revision is 2: the -11 mV offset moved the ratio 856 -> 855.4 (-0.065 %, i.e.
 * 12 mV at 18 V against a 200 mV margin), so the line stands but was NOT re-measured. */
#define PD_RANGE_MEASURED_CAL 2
#define PD_RANGE_MARGIN_MV 200  /* regulation headroom below the measured maximum */

/* BENCH PROBE: let the DECLARED range answer, not the predictive model.
 *
 * Set while the operator measures the real transfer line between 39 V and 42 V. The re-fitted model
 * (PD_RANGE_GAIN_LINKED_NUM = 2060) answers 19809*2060/1000 - 200 = 40606 mV at 19.8 V input, so a
 * 42000 mV setpoint would be refused by SET_TARGET, by the START gate and by the start precheck -
 * and the measurement could not be taken. A model that is being TESTED must not be the thing that
 * refuses the test.
 *
 * 1 = PD_PowerAllowedVmaxMv() returns the declared ceiling for any input inside the window, so every
 *     caller is covered by one branch. 0 = the measured line answers, as it must for a release.
 *
 * WHAT THIS WITHDRAWS: the pre-emptive "this input cannot reach that target" refusal, and the
 * runtime range servo in PD_PowerTick that descends when input sag makes a running target
 * unreachable. REGULATION FAILED still latches PD_FAULTS_INTERNAL and stops the stage - proven on the
 * bench in log 28 - so a target that cannot be reached ends in a clean latched stop, not a runaway.
 *
 * ONE THING THE OPERATOR MUST EXPECT: the actuator runs out at about 40.9 V. Log 28 has A pinned at
 * PD_CMP_A_MAX and B on the 30 % floor simultaneously with VOUT stuck at 40899 mV. So 39 V and 40 V
 * give real steady points, and 41..42 V will read as SATURATION until the B floor comes down. That is
 * information, not a defect - and it is the number that decides the B floor for 44 V.
 *
 * check_configuration.py --release refuses 1 and PD_BoardInit() announces it at boot. */
#define PD_RANGE_PROBE 1
#define PD_RANGE_RESERVE_COUNTS 109u /* topology model only (upper bound, see above) */
#define PD_RANGE_LOSS_MARGIN_MV 150  /* topology model only (upper bound, see above) */
#define PD_VIN_RANGE_HOLD_MS 10u     /* accepted target outside the range this long -> FAULT */
/* INPUT WINDOW 19.0 .. 21.0 V: the operator's constraint is 20 +/- 1 V and nothing wider.
 *
 * This is a NARROWING and it costs something real: the 24 V bench point was taken at 19.369 V and
 * the 20.8 V points at 20.77 V, so both stay inside, but the old 17 V experiments do not. The
 * window is also what bounds everything else - outside it the stage refuses before any protection
 * gets involved, which is the strongest possible answer to "the input must not be exceeded".
 *
 * Hardware note that the narrower window finally satisfies: C16/C17 are 330 uF /
 * JVJ35V330M10X10 rated 35 V. The design always kept them under 66 % of that (VIN <= 23 V); at
 * 21 V they sit at 60 %. */
#define PD_VIN_MIN_MV 19000
#define PD_VIN_MAX_MV 21000
#define PD_LOOP_HZ 1000u
#define PD_LOOP_DT 0.001f
/* ================== voltage-channel conversion calibration (0.3.22) ========
 * The ADC reference is shared by every channel, but the CURRENT channels must
 * keep the old conversion basis for now: pd_board.c:current() subtracts a fixed
 * 1650 mV sensor zero, and the IOUT/IL zeros in pd_zero.c are stored in the old
 * "ADC code x PD_VREF_MV" unit. Replacing PD_VREF_MV globally would give the
 * near-mid-scale IIN reading an instant ~-75 mA offset and change the input
 * over-current decision. So only VIN and VOUT use the measured reference below;
 * PD_VREF_MV stays the basis of every current conversion and of the zero store.
 *
 * Bench: VDDA/VREF+ measured about 3.285 V (15 V operating point). Using 3285
 * instead of 3300 shifts the reported voltage by -0.455 %: a 15.000 V reading
 * becomes 14.932 V, which still leaves about -0.55 % against the 14.85 V the
 * multimeter showed - i.e. this correction does NOT explain the whole error and
 * the residue needs the gain/offset calibration below. Keep PD_CALIBRATED = 0.
 */
#define PD_VOLTAGE_SCALE_REF_MV 3285u
/* Optional per-channel gain/offset hook, INERT in this version (gain 1, offset
 * 0). Fill only from at least two same-board, same-instrument voltage points;
 * never from unrelated multimeter readings. Order of operations: raw code ->
 * reference correction (3285) -> gain -> offset. Keep the reference correction
 * and the gain correction separate, never apply both to the same factor.
 * Use tools/apply_voltage_calibration.py rather than editing this by hand, and note
 * the COUPLING WITH THE VOUT CALIBRATION documented above PD_RANGE_GAIN_NUM/DEN: the
 * calibrated VOUT is what feeds the control loop, so PD_RANGE_GAIN_* and
 * PD_RANGE_MEASURED_CAL must be updated in the same build. The compiler enforces it. */
#define PD_VOUT_CAL_GAIN_NUM 1000
#define PD_VOUT_CAL_GAIN_DEN 1000
#define PD_VOUT_CAL_OFFSET_MV -11
#define PD_VIN_CAL_GAIN_NUM 1000
#define PD_VIN_CAL_GAIN_DEN 1000
#define PD_VIN_CAL_OFFSET_MV 0
#define PD_VOUT_LPF_ALPHA 0.35f
#define PD_IOUT_LPF_ALPHA 0.12f
/* Compensator: C(s) = (KP + KI/s) * 1/(1 + s/wp), wp = 2*pi*300 Hz.
 * Plant duty->VOUT gain is VIN/0.75 = 26.7 at 20 V. The 150 Hz crossover,
 * 15 Hz zero and >100 deg phase margin are DESIGN TARGETS computed from the
 * ideal plant, not measurements: no closed-loop sweep or load-step test has
 * been run on hardware. Loop 1 (CV) / loop 2 (CC) from the PID page are NOT
 * wired to this compensator. */
#define PD_CV_KP 0.038f
#define PD_CV_KI 3.6f
#define PD_CV_POLE_ALPHA 0.653f
#define PD_SS_MS 400u
/* Two independent down rates (0.3.20): the regulation down-ramp and the STOP
 * ramp were the same constant before. Values are unchanged, only split. */
#define PD_STOP_RAMP_MV_MS 40
#define PD_RUN_RAMP_DOWN_MV_MS 40
#define PD_PRECHARGE_MS 3u
/* Down-transition supervision (0.3.20): completion band, how long the feedback
 * must stay inside it, and the timeout budget = ramp time + settling, floored. */
#define PD_DOWN_BAND_MV 300
#define PD_DOWN_BAND_SAMPLES 50u
#define PD_DOWN_SETTLE_MS 1500u
#define PD_DOWN_TIMEOUT_MIN_MS 3000u
#define PD_CC_GAIN_MV_PER_MA_S 600.0f
#define PD_CC_DEADBAND_MA 10
#define PD_CC_ACTIVE_MV 100
#define PD_OVP_MARGIN_MV 1500
#define PD_OVP_RATIO_NUM 115
#define PD_OVP_RATIO_DEN 100
/* A single ADC sample must never latch a protection. The gate-enable transient
   on this board disturbs every sense channel for ~160 us (measured 2026-09-09),
   so OVP and the input window require the condition to persist. */
#define PD_OVP_HOLD_MS 10u
#define PD_INPUT_HOLD_MS 10u
/* Absolute OVP ceiling. The tracking threshold below may not exceed this, so a
 * ramp can never widen the protection without bound. It is derived from the
 * SPECIFIED maximum output and the existing ratio:
 *   0.3.20..0.3.21: 15000 * 115/100 = 17250 mV (deliberately NOT raised then)
 *   0.3.22 (this version): 18000 * 115/100 = 20700 mV - THE CEILING IS RAISED
 * by design because the qualified output range moves from 15 V to 18 V. The
 * release notes must state this; do not repeat the old "ceiling not raised"
 * wording. 20700 mV is a software tracking cap, NOT a permitted output setting
 * and not a promise about what the load tolerates.
 * Hardware margin: output bank U5/U6/U7 = 220 uF/100 V and the FETs are 100 V
 * parts, so 20.7 V is far inside the hardware rating; the binding limit is the
 * allowed output range, not the components. The 10 ms OVP persistence and the
 * hardware OC shutdown are unchanged and are not a substitute for each other. */
#define PD_OVP_ABS_MAX_MV (PD_BUCK_VMAX_MV * PD_OVP_RATIO_NUM / PD_OVP_RATIO_DEN)
/* 20000 x 1.15 = 23000 mV. It is still capped by PD_OUT_CAP_RATED_MV through the
 * configuration gate, and the 10 ms OVP persistence plus the hardware OC shutdown are
 * unchanged. Raising this ceiling is a protection change and the release notes must say
 * so, exactly as they did when 0.3.23 moved it from 17250 to 20700. */
/* Historical diagnostic-only software blanking. The release build leaves
   hardware OC active; the startup event has NOT been proved to be noise. */
#define PD_OC_BLANK_MS 50u
#define PD_OC_LATCH_NUM 3
#define PD_OC_LATCH_DEN 2
#define PD_OC_LATCH_MARGIN_MA 300
#define PD_OC_LATCH_MS 50
/* Input over-current backstop (hard stop in EVERY build). The board had no
   input-current protection at all, so a bridge fault silently collapsed the
   bench supply into current limit with no firmware evidence. The limit is
   ADAPTIVE: twice the power-balance estimate plus a margin, floored at
   PD_INPUT_OC_MIN_MA, because a 500 mA-limited supply can never reach a fixed
   600 mA threshold and a fixed value cannot serve both 2 V/0.1 A and 15 V/2 A. */
#define PD_INPUT_OC_MIN_MA 250
#define PD_INPUT_OC_MARGIN_MA 150
#define PD_INPUT_OC_RATIO_NUM 2
#define PD_INPUT_OC_RATIO_DEN 1
#define PD_INPUT_OC_MS 10u
#define PD_REG_FAIL_MS 500
#define PD_REG_FAIL_MARGIN_MV 800
/* IL software trip. Was 0 (disabled) because the IL path was not trusted; it is enabled now
 * because masking FLT3 (below) removes the ALERT_N hardware backstop and leaves OC_N alone.
 *
 * 9000 mA against the inductor's ~10 A saturation rating, with a consecutive-sample count so a
 * single glitch cannot stop the stage: the injected rank samples IL once per PD_TE_PERIOD, and a
 * genuine over-current holds for many periods while a coupling spike does not.
 *
 * WHAT THIS DOES NOT SEE: the injected sample is taken at one fixed phase of the switching period,
 * so it is not a peak detector. A spike that lives between samples is invisible to it - and that is
 * precisely the limitation that makes the masking decision a trade, not a free win. */
/* Is the TMCS1133 inductor-current sensor fitted? THIS BOARD: being removed.
 *
 * The TMCS1133B4A sits in series between the inductor and SW2 (hardware map, section 3) - i.e. in
 * the switching-node path, where dv/dt is extreme. The operator reports its interference is already
 * significant at NO LOAD, which is structurally expected in that position rather than a wiring
 * fault, and has decided to remove it.
 *
 * WHAT GOES WITH IT - this is not just IL. Confirmed by the operator: OC_N is a TMCS1133 output too.
 *     IL       ADC1_IN1 / PA0   the injected-group measurement
 *     OC_N     PA12 -> HRTIM FLT1
 *     ALERT_N  PB10 -> HRTIM FLT3
 * Both of CH1's hardware fault inputs disappear, so this withdraws the FAST hardware overcurrent
 * backstop entirely. What remains, and it must be said plainly:
 *     - the IOUT slow current limit (R15 + INA296A4), software, at the 1 kHz loop rate
 *     - OVP, the input window, PD_LOOP_STALE_MS, the sampling-health gate, COMM_TIMEOUT
 *     - whatever the bench supply's own current limit does
 * There is NO hardware trip left on CH1. A short is then handled in milliseconds, not nanoseconds,
 * on a converter that can reach 44 V. The recommended hardware follow-up is a comparator on the
 * IOUT shunt feeding the now-free FLT1 input.
 *
 * The two pins must ALSO be terminated on the board (pull-up to 3.3 V) once the sensor is gone.
 * This firmware masks them, so a floating pin will not stop the stage - but a floating CMOS input
 * drifts, its HRTIM fault flag can chatter, and a pull-up costs one resistor.
 *
 * 0 = sensor removed: IL is not measured nor published, and OC_N/ALERT_N are ignored for CH1.
 * 1 = sensor fitted: normal operation. check_configuration.py --release refuses 0. */
#define PD_IL_SENSOR_FITTED 0

/* The debounced IL trip needs an IL measurement to trip on. Zeroed with the sensor rather than left
 * at an unreachable 9 A, so no log line can advertise a protection that no longer exists. */
#define PD_IL_TRIP_MA 0
#define PD_IL_TRIP_COUNT 8u
#if !PD_IL_SENSOR_FITTED && PD_IL_TRIP_MA > 0
#error "PD_IL_TRIP_MA must be 0 when PD_IL_SENSOR_FITTED = 0: there is no IL signal to trip on"
#endif

/* Mask the ALERT_N (HRTIM FLT3) HARDWARE shutdown on CH1's two bridges, keeping the flag and the
 * log line. Mechanism retained, now SET TO 0 - the protection is restored.
 *
 * HISTORY, because the masking turned out to be a mistake worth recording rather than deleting:
 * this was set to 1 in 0.3.38 to stop ALERT_N false-tripping at 33 V, whose HRTIM filter was
 * HRTIM_FAULTFILTER_NONE. Two things were then found on the bench:
 *   1. Masking was INEFFECTIVE. It cleared FLT3EN (the hardware output shutdown, pd_board.c) but
 *      PD_BoardFaultIRQ() software-latches on the FLT3 FLAG with no mask check at all, calls
 *      PD_LatchFault(), and that stops the stage anyway. The shutdown had merely moved from
 *      hardware to software. Log 26 proves it: HRTIM FLT m=0x04 (CH1ALR) with FSTOP f=02 (=
 *      SENSOR_ALERT) and st=5 (= FAULT_LATCHED).
 *   2. The real defect was the missing filter. fFLTS = fHRTIM = 5.442 GHz under
 *      HRTIM_FAULTPRESCALER_DIV1 + MUL32, so HRTIM_FAULTFILTER_NONE validated a fault on a single
 *      0.18 ns sample - a nanosecond coupling glitch is enough. hrtim.c now sets
 *      HRTIM_FAULTFILTER_15 (fFLTS/32, N=8 = 47.1 ns), the strongest debounce available.
 * With a filter in place the reason to mask is gone, so this is 0 again and FLT3EN is left alone.
 *
 * KEEP THE SWITCH: it stays a named, gate-checked knob for bench work. check_configuration.py
 * --release still refuses 1. */
#define PD_MASK_ALERT_NOUT 0

/* Is a second power board fitted to CH2? THIS BOARD: no.
 *
 * The CH2 fault inputs (PA15 = OC_N, PB11 = ALERT_N) are active-low with no pull-up on an unpopulated
 * channel, so they sit LOW. PD_Poll() polls PD_PortFaultPins(2) every millisecond and latches
 * whatever it reports. The loop already rate-limited its LOG for CH2 to once per 30 s - the comment
 * there says "CH2 no power board: pins float low" - but PD_LatchFault() sat OUTSIDE that rate limit
 * and ran every pass.
 *
 * PD_LatchFault() is not a flag setter. It calls PD_PortStop(), PD_PowerAbort(), and pins the channel
 * at PD_STATES_FAULT_LATCHED with enabled = ready = 0. So with no board fitted:
 *   - PD_FAULTS_SENSOR_ALERT (bit 2) was re-latched on CH2 every millisecond, forever, and bit 2 is
 *     what PowerDesk renders as "传感器诊断". The GUI showed a permanent sensor-fault prompt for a
 *     channel that has no sensor, which is exactly what the operator reported on 2026-09-19 while
 *     the zero calibration was passing (ZERO st=3 why=0) and CH1 was healthy.
 *   - CH2 could never leave FAULT_LATCHED, so fitting a board later would not have brought it up.
 * It was present in EVERY bench log (18 through 26); "FAULT PIN: CH1" never appeared once.
 *
 * 0 = no CH2 board: CH2's fault pins are not polled and its fault bit is not latched.
 * 1 = a CH2 board is fitted: normal monitoring and latching.
 *
 * WHAT IS GIVEN UP: CH2's fault monitoring. There is nothing to protect on a channel with no
 * hardware, but this is still a withdrawable monitor, so it is treated like one: PD_BoardInit()
 * prints a banner naming it at boot (nothing is concealed), and check_configuration.py --release
 * REFUSES a release build with PD_CH2_FITTED = 0. A release for a two-channel board must set 1.
 *
 * CH1 IS NOT AFFECTED: PD_PortStop(2) only clears GPIOB pin 2 and ODISR 0x00f0 (CH2's outputs);
 * CH1's enable (PA3) and ODISR 0x000f are untouched. */
#define PD_CH2_FITTED 0
#define PD_GATE_TEST_MS 600u
#define PD_GATE_TEST_VIN_MAX_MV 5000
#define PD_GATE_TEST_VOUT_MAX_MV 1000
/* DIAGNOSTIC BUILD ONLY. The 0.3.12-0.3.16 investigation used these switches to
 * characterise CH1 OC_N -> HRTIM FLT1. The illegal compare value was a confirmed
 * code defect, but does not prove a particular FET conduction path occurred.
 * The release build keeps the hardware fault protection ENABLED:
 *   PD_DIAG_MASK_OC  1 = FLT1 output shutdown masked, flag + one-shot interrupt
 *                        kept, OC_N sampled once per injected IL pair (125 kHz)
 *   PD_OVERRIDE_NO_LATCH 1 = latching protections log-only (bench override)
 * Both must be 0 for any build that touches real hardware. check_configuration.py
 * --release fails if either is 1. */
#ifndef PD_DIAG_MASK_OC
#define PD_DIAG_MASK_OC 0
#endif
/* Injected-IL samples (one per PD_TE_PERIOD, i.e. 8 us at 125 kHz) of OC_N low
 * before the diag software backstop trips: 100 samples = 800 us. */
#define PD_DIAG_OC_LOW_SAMPLES 100u
/* BENCH OVERRIDE BUILD ONLY (Release/v0.3.14-override), requested to observe
 * one real power-up run. Latching protections become log-only: the stage keeps
 * running instead of stopping. Downgraded: ADC fault, input window, slow OC
 * latch, regulation failure, fault pins, HRTIM FLT3 (ALERT) output shutdown.
 * STILL ACTIVE, deliberately:
 *   - COMM_TIMEOUT: an unattended stage must not keep running if the host dies;
 *   - OVP at PD_OVERRIDE_OVP_MV (raised, not removed): protects the output bank
 *     (netlist: U5/U6/U7 = 220 uF/100 V, NOT 25 V) and the load from a runaway
 *     duty;
 *   - duty clamp PD_DUTY_MAX: with the 0.70 ceiling VOUT cannot exceed
 *     VIN*0.928 = 18.6 V at 20 V (see the range block above);
 *   - the continuous OC_N low backstop (a genuine over-current);
 *   - PD_LOOP_STALE_MS: stale sampling now stops the stage in every build.
 * The bench supply current limit is the only fast short-circuit backstop:
 * keep it at 0.5 A, keep the 20 ohm load, never leave it unattended.
 * The production configuration is 0. */
#ifndef PD_OVERRIDE_NO_LATCH
#define PD_OVERRIDE_NO_LATCH 0
#endif
#define PD_OVERRIDE_OVP_MV 8000
/* Independent stop backstop; normal sampling health checks are stricter. */
/* ADC-fault debounce (re-applied after the 0.3.34 revert).
 *
 * adc_errors is a running diagnostic total and MUST NOT gate anything on its own: it is set by
 * four different ISR paths and has no clearing path, so one transient - which is exactly what a
 * gate-enable edge can produce - used to disable the output for the rest of the power cycle, with
 * CLEAR_FAULT unable to help (it reads PD_PortSamplingHealthy() too, so it answered NOT_READY).
 * The operator's only recovery was a power cycle.
 *
 * PD_ADC_ERR_STREAK: how many CONSECUTIVE error events are tolerated before the sampling is
 * judged broken. Each event in a self-clearing transient resets the count, so a single glitch is
 * absorbed while a genuine failure - for which the errors repeat or nothing new arrives - still
 * latches after three events.
 * PD_ADC_ERR_STREAK_MS: the window those events must fall in. A dead ADC produces no sample pair
 * at all, which the freshness checks below already catch within PD_LOOP_STALE_MS, so this window
 * only has to be long enough not to merge unrelated glitches.
 */
#define PD_ADC_ERR_STREAK 3u
#define PD_ADC_ERR_STREAK_MS 50u
#define PD_LOOP_STALE_MS 20u
/* Consecutive 1 kHz ticks in which a channel's regular group delivered nothing before its VIN /
 * VOUT / IOUT bits are withdrawn. 20 ticks = 20 ms, the same order as the age test it replaces, so
 * a stopped conversion is still caught quickly - but a HEALTHY one can no longer be refused for a
 * reason that the code and its own log disagree about. */
#define PD_SLOW_STALE_TICKS 20u
/* Log payload budget: PD_LOG_MESSAGE_OFFSET is a 96-byte field INCLUDING the
 * terminator, so a formatted line must stay at or below this many characters.
 * Every formatted line is now BOUNDED at compile time instead of being counted by
 * hand: values whose physical range is known go through PD_LOG_VALUE, and
 * tools/check_log_lengths.py recomputes the worst case of every formatted line in
 * the tree (it is a gate in check_configuration.py --release). 0.3.22 shipped two
 * 1 Hz lines longer than this and lost their tails on the wire.
 * NEVER raise this without checking pd_protocol.h: it must stay PD_LOG_SIZE - 1. */
#define PD_LOG_TEXT_MAX 95u
/* Bounded rendering for diagnostic values. Anything above the cap prints as 1000
 * ("+"), which is honest - it says "at or beyond the cap" - and keeps the field
 * width predictable instead of letting a corrupted ADC read push a line past the
 * end of the log field. A negative value (possible on a signed millivolt reading)
 * also prints as the "+" marker, so the result is never wider than four digits.
 * The parameter type is int32_t on purpose: every caller passes either an int32_t
 * or something that promotes to it, and the result is unsigned int, i.e. what %u
 * expects - a macro that compared mixed signedness would not build with
 * -Wall -Wextra -Werror, which is how this was caught. */
#define PD_LOG_VALUE_MAX 999u
static inline unsigned PD_LogValue(int32_t value)
{
    if (value < 0 || value > (int32_t)PD_LOG_VALUE_MAX)
        return PD_LOG_VALUE_MAX + 1u;
    return (unsigned)value;
}
#define PD_LOG_VALUE(v) PD_LogValue(v)
/* Single source for the reported firmware version (INFO reply + boot log).
 * 0.3.32-SATCLEAN. The INFO reply writes it at offset 4 and the device ID at offset 36, so the
 * field holds 31 characters; the string and the release directory name must have the same
 * length after the "G474-BUCK-" prefix, which check_configuration.py --release enforces.
 * tools/check_configuration.py --release <name> asserts this string matches the release
 * being packaged: 0.3.28 was first packaged with this still reading 0.3.27-CALFIX, i.e. the
 * HEX would have flashed correctly and then REPORTED the old version - impossible to tell
 * from telemetry whether the new modulator was running. Do not hand-edit one without the
 * other. */
/* 0.3.34 carried the same string through the ADC-debounce change, so there was no way to
 * tell from the log which build was running. Bumped with the range work. */
/* 0.3.38 carries PD_MASK_ALERT_NOUT = 1, i.e. one protection is WITHDRAWN (only its flag and
 * logging remain). The string says so, because the handshake line is the only thing standing
 * between a masked build and someone treating it as a release.
 * 0.3.41-NOWRAP: every sample-age test is now underflow-safe (sample_age_ms). It still masks
 * ALERT_N, so it is a MONITOR build and check_configuration.py --release still refuses it.
 * 0.3.42-CH2OFF: PD_CH2_FITTED = 0, so CH2's floating fault pins no longer latch a permanent
 * SENSOR_ALERT ("传感器诊断") and no longer hold CH2 in FAULT_LATCHED. That WITHDRAWS CH2's fault
 * monitor, so this build has TWO withdrawn protections and --release refuses it on both counts.
 * 0.3.43-FLTFILT: the HRTIM fault inputs get HRTIM_FAULTFILTER_15 (47.1 ns) and PD_MASK_ALERT_NOUT
 * returns to 0, so ALERT_N's real protection is BACK and only CH2's monitor stays withdrawn.
 * 0.3.44-NOIL: PD_IL_SENSOR_FITTED = 0. The TMCS1133 is being removed, and it was the source of
 * IL, OC_N (FLT1) AND ALERT_N (FLT3) - so CH1 now has NO hardware overcurrent trip. The string says
 * NOIL rather than something neutral because the handshake line is the only thing standing between
 * a build with no hardware OC protection and someone treating it as an ordinary release.
 * 0.3.45-NOILFIX: 0.3.44 masked CH2's fault report, which un-suppressed the fault-interrupt re-arm
 * that the channel's stuck-low pins had been holding off by accident. FLT2/FLT4 were then re-armed
 * every millisecond against pins that were still low: 986 identical log lines in two minutes, and
 * PowerDesk declared the session dead one second after the handshake. Fault hardware presence is now
 * ONE predicate (channel_fault_hw) shared by the mask, the re-arm and the interrupt arming. */
#define PD_FW_VERSION "G474-BUCK-0.3.51-NTC"
#endif
