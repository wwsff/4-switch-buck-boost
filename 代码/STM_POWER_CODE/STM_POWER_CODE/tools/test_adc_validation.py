"""Run the actual startup ADC validator against ST LL headers and RAM registers.

This checks register encoding, not ADC hardware or interrupt timing. Windows
allocates registers below 4 GB because the MCU LL pointer helpers are 32-bit.
Pass --before-ref <commit> to demonstrate rejection by the former validator.
"""
from pathlib import Path
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
build = root / 'tests/build'
build.mkdir(exist_ok=True)
board = (root / 'App/pd_board.c').read_text()
if len(sys.argv) == 3 and sys.argv[1] == '--before-ref':
    board = subprocess.check_output(['git', '-c', f'safe.directory={root.as_posix()}',
        'show', sys.argv[2] + ':App/pd_board.c'], cwd=root, text=True)
validator = board[board.index('static void validate_adc('):board.index('void PD_BoardInit(void)')]
source = r'''
#include <windows.h>
#undef LPTR
#undef ERROR
#include <stdio.h>
#include "stm32g4xx_hal.h"
static int failures;
static void require_config(uint8_t ok) { if (!ok) failures++; }
''' + validator + r'''
int main(void) {
    static const uint32_t channels[2][5] = {
        {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_3},
        {ADC_CHANNEL_5, ADC_CHANNEL_11, ADC_CHANNEL_17, ADC_CHANNEL_13, ADC_CHANNEL_3}};
    static const uint32_t ranks[5] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2,
        ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5};
    static const uint32_t times[5] = {ADC_SAMPLETIME_47CYCLES_5, ADC_SAMPLETIME_47CYCLES_5,
        ADC_SAMPLETIME_24CYCLES_5, ADC_SAMPLETIME_92CYCLES_5, ADC_SAMPLETIME_92CYCLES_5};
    ADC_HandleTypeDef a = {0};
    ADC_TypeDef saved;
    int ch, i, scenario, checks = 0;
    a.Instance = VirtualAlloc((void *)0x10000000, 4096, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!a.Instance || (uintptr_t)a.Instance > UINT32_MAX) return 2;
    a.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
    a.Init.NbrOfConversion = 5;
    a.Init.DiscontinuousConvMode = ENABLE;
    a.Init.NbrOfDiscConversion = 1;
    a.Init.ContinuousConvMode = DISABLE;
    a.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
    a.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    a.Init.DMAContinuousRequests = ENABLE;
    for (ch = 0; ch < 2; ch++) {
        uint32_t il = ch ? ADC_CHANNEL_2 : ADC_CHANNEL_1;
        memset(a.Instance, 0, sizeof(*a.Instance));
        for (i = 0; i < 5; i++) {
            LL_ADC_REG_SetSequencerRanks(a.Instance, ranks[i], channels[ch][i]);
            LL_ADC_SetChannelSamplingTime(a.Instance, channels[ch][i], times[i]);
        }
        LL_ADC_INJ_SetSequencerRanks(a.Instance, ADC_INJECTED_RANK_1, il);
        LL_ADC_SetChannelSamplingTime(a.Instance, il, ADC_SAMPLETIME_24CYCLES_5);
        LL_ADC_INJ_SetTriggerSource(a.Instance, ADC_EXTERNALTRIGINJEC_HRTIM_TRG2);
        saved = *a.Instance;
        for (scenario = 0; scenario < 5; scenario++) {
            *a.Instance = saved;
            if (scenario == 1) LL_ADC_REG_SetSequencerRanks(a.Instance, ranks[0], ADC_CHANNEL_4);
            if (scenario == 2) LL_ADC_INJ_SetSequencerRanks(a.Instance, ADC_INJECTED_RANK_1, ADC_CHANNEL_4);
            if (scenario == 3) LL_ADC_SetChannelSamplingTime(a.Instance, channels[ch][0], ADC_SAMPLETIME_2CYCLES_5);
            if (scenario == 4) LL_ADC_INJ_SetTriggerSource(a.Instance, LL_ADC_INJ_TRIG_SOFTWARE);
            failures = 0;
            validate_adc(&a, channels[ch], il);
            if ((failures == 0) != (scenario == 0)) {
                printf("FAIL CH%d scenario=%d rejected_checks=%d\n", ch + 1, scenario, failures);
                VirtualFree(a.Instance, 0, MEM_RELEASE);
                return 1;
            }
            checks++;
        }
    }
    VirtualFree(a.Instance, 0, MEM_RELEASE);
    printf("PASS %d cases: valid ADC1/2 accepted; wrong regular/injected channel, sampling time and trigger rejected.\n", checks);
    return 0;
}
'''
test = build / 'adc_validation.c'
test.write_text(source)
gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')
includes = ['Core/Inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
            'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include']
command = [str(gcc), '-std=c99', '-O2', '-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast',
           '-DSTM32G474xx', '-DUSE_HAL_DRIVER']
for include in includes:
    command += ['-I', str(root / include)]
exe = build / 'adc_validation.exe'
subprocess.run(command + [str(test), '-o', str(exe)], env=env, check=True)
subprocess.run([str(exe)], check=True)
