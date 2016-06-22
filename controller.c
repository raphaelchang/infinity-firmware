#include "controller.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "comm_usb.h"
#include "encoder.h"
#include "transforms.h"
#include "zsm.h"

#define SYSTEM_CORE_CLOCK       168000000
#define DEAD_TIME_CYCLES          1000
#define PWM_FREQ                    20000.0

#define SET_DUTY(duty1, duty2, duty3) \
        TIM1->CR1 |= TIM_CR1_UDIS; \
        TIM1->CCR1 = (int)(duty1 * TIM1->ARR); \
        TIM1->CCR2 = (int)(duty2 * TIM1->ARR); \
        TIM1->CCR3 = (int)(duty3 * TIM1->ARR); \
        TIM1->CCR4 = TIM1->ARR - 2; \
        TIM1->CR1 &= ~TIM_CR1_UDIS;

#define CHECK_CURRENT(adc) (adc > 500 && adc < 4096 - 500)

static volatile ZSMMode zsm_mode = SINUSOIDAL;
static volatile ControllerState state = ZEROING;
static volatile ControllerFault fault = NO_FAULT;
static volatile uint16_t ADC_Value[3];

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
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUPDR_FLOATING);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_DeInit(TIM1);
    TIM1->CNT = 0;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Time Base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2; // compare flag when upcounting
    TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / 2 / PWM_FREQ;
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
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, ADC_SampleTime_15Cycles);

    // ADC2 regular channels
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);

    // ADC3 regular channels
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);

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

static int adc1;
static int adc2;
static int adc3;
static int e = 0;
static float angle = 0;

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
    switch(state)
    {
        case STOPPED:
            break;
        case RUNNING:
            angle = encoder_get_angle();
            // float a, b, c, alpha, beta;
            // transforms_inverse_park(0, 0.2, angle, &alpha, &beta);
            // transforms_inverse_clarke(alpha, beta, &a, &b, &c);
            // controller_apply_zsm(&a, &b, &c);
            // SET_DUTY(a, b, c);
            break;
        case ZEROING:
            if (e++ < 4000)
            {
                float a, b, c, alpha, beta;
                transforms_inverse_park(0.25, 0, 0, &alpha, &beta);
                transforms_inverse_clarke(alpha, beta, &a, &b, &c);
                controller_apply_zsm(&a, &b, &c);
                SET_DUTY(a, b, c);
                angle = encoder_get_angle();
            }
            else
            {
                e = 0;
                state = RUNNING;
            }
            break;
    }

}

void controller_print_adc(void)
{
    // if (state == ZEROING)
    {
        USB_PRINT("%f\n", angle);
    }
    // USB_PRINT("%d %d %d\n", adc1, adc2, adc3);
}

void controller_apply_zsm(float *a, float *b, float *c)
{
    switch(zsm_mode)
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
