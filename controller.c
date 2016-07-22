#include "controller.h"
#include "ch.h"
#include "hal.h"
#include "hw_conf.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "comm_usb.h"
#include "encoder.h"
#include "transforms.h"
#include "zsm.h"
#include "datatypes.h"
#include "config.h"

#define SYSTEM_CORE_CLOCK       168000000
#define DEAD_TIME_CYCLES        60

#define SET_DUTY(duty1, duty2, duty3) \
        TIM1->CR1 |= TIM_CR1_UDIS; \
        TIM1->CCR1 = (int)(duty1 * TIM1->ARR); \
        TIM1->CCR2 = (int)(duty2 * TIM1->ARR); \
        TIM1->CCR3 = (int)(duty3 * TIM1->ARR); \
        TIM1->CCR4 = TIM1->ARR - 2; \
        TIM1->CR1 &= ~TIM_CR1_UDIS;

#define CHECK_CURRENT(adc) (adc > 500 && adc < 4096 - 500)

static volatile Config *config;
static volatile ControllerState state = RUNNING;
static volatile ControllerFault fault = NO_FAULT;
static volatile uint16_t ADC_Value[3];

static volatile int adc1;
static volatile int adc2;
static volatile int adc3;
static volatile float currA;
static volatile float currB;
static volatile float currC;
static volatile float currD;
static volatile float currQ;
static volatile int e = 0;
static volatile float angle = 0;
static volatile float a, b, c;
static volatile float lastDutyCycle[3];

CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    controller_update();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void controller_init(void)
{
    config = config_get_configuration();
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_DeInit(TIM1);
    TIM1->CNT = 0;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Time Base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2; // compare flag when upcounting
    TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / 2 / config->pwmFrequency;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1; // Only generate update event on underflow

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Channel 1, 2 and 3 Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = DEAD_TIME_CYCLES;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // Clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

    // DMA for the ADC
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 3;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    // DMA2_Stream0 enable
    DMA_Cmd(DMA2_Stream4, ENABLE);

    // ADC Common Init
    // Note that the ADC is running at 42MHz, which is higher than the
    // specified 36MHz in the data sheet, but it works.
    ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_InjecSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // Channel-specific settings
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    // ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
    // ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = 0;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Init(ADC3, &ADC_InitStructure);

    // Enable Vrefint channel
    // ADC_TempSensorVrefintCmd(ENABLE);

    // Enable DMA request after last transfer (Multi-ADC mode)
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // Injected channels for current measurement at end of cycle
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
    // ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
    // ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);
    ADC_InjectedSequencerLengthConfig(ADC3, 1);
    // ADC_InjectedSequencerLengthConfig(ADC2, 2);

    // ADC1 regular channels
    ADC_InjectedChannelConfig(ADC1, CURR_A_CHANNEL, 1, ADC_SampleTime_15Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, ADC_SampleTime_15Cycles);

    // ADC2 regular channels
    ADC_InjectedChannelConfig(ADC2, CURR_B_CHANNEL, 1, ADC_SampleTime_15Cycles);

    // ADC3 regular channels
    ADC_InjectedChannelConfig(ADC3, CURR_C_CHANNEL, 1, ADC_SampleTime_15Cycles);

    // Interrupt
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    nvicEnableVector(ADC_IRQn, 4);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    // Enable ADC2
    ADC_Cmd(ADC2, ENABLE);

    // Enable ADC3
    ADC_Cmd(ADC3, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    SET_DUTY(0, 0, 0);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    WWDG_SetPrescaler(WWDG_Prescaler_1);
    WWDG_SetWindowValue(255);
    WWDG_Enable(100);
}

/* Updates the controller state machine. Called as an interrupt handler on new ADC sample. */
void controller_update(void)
{
    WWDG_SetCounter(100);
    adc1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    adc2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    adc3 = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1);
    if (fault == OVERCURRENT || !(CHECK_CURRENT(adc1) && CHECK_CURRENT(adc2) && CHECK_CURRENT(adc3)))
    {
        SET_DUTY(0, 0, 0);
        fault = OVERCURRENT;
        return;
    }
    currC = ((int16_t)adc1 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    currB = ((int16_t)adc2 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    currA = ((int16_t)adc3 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    // Check for ADC measurements that happened when the low side PWM was too short
    if (lastDutyCycle[0] > 0.9)
    {
        currA = -(currB + currC);
    }
    else if (lastDutyCycle[1] > 0.9)
    {
        currB = -(currA + currC);
    }
    else if (lastDutyCycle[2] > 0.9)
    {
        currC = -(currA + currB);
    }
    // If all are good, use all the balanced current assumption to use all three channels to improve each channel's measurement
    else if (lastDutyCycle[0] <= 0.95 && lastDutyCycle[1] <= 0.95 && lastDutyCycle[2] <= 0.95)
    {
        currA = currA / 2.0 - (currB + currC) / 2.0;
        currB = currB / 2.0 - (currA + currC) / 2.0;
        currC = currC / 2.0 - (currA + currB) / 2.0;
    }
    switch(state)
    {
        case STOPPED:
            break;
        case RUNNING:
            angle = encoder_get_angle();
            float alpha, beta;
            transforms_inverse_park(0, 0.1, angle, &alpha, &beta);
            transforms_inverse_clarke(alpha, beta, &a, &b, &c);
            controller_apply_zsm(&a, &b, &c);
            SET_DUTY(a, b, c);
            lastDutyCycle[0] = a;
            lastDutyCycle[1] = b;
            lastDutyCycle[2] = c;
            transforms_clarke(currA, currB, currC, &alpha, &beta);
            transforms_park(alpha, beta, angle, &currD, &currQ);
            break;
        case ZEROING:
            if (e++ < 4000)
            {
                float alpha, beta;
                transforms_inverse_park(0.2, 0, 0, &alpha, &beta);
                transforms_inverse_clarke(alpha, beta, &a, &b, &c);
                controller_apply_zsm(&a, &b, &c);
                SET_DUTY(a, b, c);
                angle = encoder_get_raw_angle();
                lastDutyCycle[0] = a;
                lastDutyCycle[1] = b;
                lastDutyCycle[2] = c;
                config->encoderZero = angle;
            }
            else
            {
                e = 0;
                state = RUNNING;
            }
            break;
    }

}

void controller_print(void)
{
    USB_PRINT("%f, %f, %f, %f, %d, %d, %d, %f, %f, %f, %f, %f\n", angle, a, b, c, adc1, adc2, adc3, currA, currB, currC, currD, currQ);
}

void controller_apply_zsm(volatile float *a, volatile float *b, volatile float *c)
{
    switch(config->zsmMode)
    {
        case SINUSOIDAL:
            zsm_sinusoidal(a, b, c);
            break;
        case MIDPOINT_CLAMP:
            zsm_midpoint_clamp(a, b, c);
            break;
        case TOP_CLAMP:
            zsm_top_clamp(a, b, c);
            break;
        case BOTTOM_CLAMP:
            zsm_bottom_clamp(a, b, c);
            break;
        case TOP_BOTTOM_CLAMP:
            zsm_top_clamp(a, b, c);
            break;
    }
}

ControllerFault controller_get_fault(void)
{
    return fault;
}

ControllerState controller_get_state(void)
{
    return state;
}
