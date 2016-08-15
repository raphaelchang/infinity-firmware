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
#include <string.h>
#include "utils.h"

#define SYSTEM_CORE_CLOCK       168000000
#define DEAD_TIME_CYCLES        60

#define SET_DUTY(duty1, duty2, duty3) \
        TIM1->CR1 |= TIM_CR1_UDIS; \
        TIM1->CCR1 = (int)(duty3 * TIM1->ARR); \
        TIM1->CCR2 = (int)(duty2 * TIM1->ARR); \
        TIM1->CCR3 = (int)(duty1 * TIM1->ARR); \
        TIM1->CCR4 = TIM1->ARR - 2; \
        TIM1->CR1 &= ~TIM_CR1_UDIS;

#define CHECK_CURRENT(adc) (adc > 500 && adc < 4096 - 500)

typedef struct {
    float edeg;
    float i_a;
    float i_b;
    float i_c;
    float i_alpha;
    float i_beta;
    float i_d;
    float i_q;
    float i_bus;
    float i_abs;
    float v_a;
    float v_b;
    float v_c;
    float v_a_norm;
    float v_b_norm;
    float v_c_norm;
    float v_alpha;
    float v_beta;
    float v_alpha_norm;
    float v_beta_norm;
    float v_d;
    float v_q;
    float v_d_norm;
    float v_q_norm;
    float v_bus;
    float integral_d;
    float integral_q;
} motor_state_t;

static volatile Config *config;
static volatile ControllerState state = RUNNING;
static volatile ControllerFault fault = NO_FAULT;
static volatile uint16_t ADC_Value[3];

static volatile motor_state_t motor_state;
static volatile int adc1_1;
static volatile int adc2_1;
static volatile int adc3_1;
static volatile int adc1_2;
static volatile int adc2_2;
static volatile int adc3_2;
static volatile float temp;
static volatile int e = 0;
static volatile float a, b, c;
static volatile float commandDutyCycle = 0.0;

static void apply_zsm(volatile float *a, volatile float *b, volatile float *c);

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
    memset((void*)&motor_state, 0, sizeof(motor_state_t));
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
    ADC_InjectedSequencerLengthConfig(ADC1, 3);
    ADC_InjectedSequencerLengthConfig(ADC2, 3);
    ADC_InjectedSequencerLengthConfig(ADC3, 2);

    // ADC1 regular channels
    ADC_InjectedChannelConfig(ADC1, CURR_A_CHANNEL, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, VSENSE_A_CHANNEL, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, VBUS_CHANNEL, 3, ADC_SampleTime_15Cycles);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 3, ADC_SampleTime_15Cycles);

    // ADC2 regular channels
    ADC_InjectedChannelConfig(ADC2, CURR_B_CHANNEL, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC2, VSENSE_B_CHANNEL, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC2, NTC_CHANNEL, 3, ADC_SampleTime_15Cycles);

    // ADC3 regular channels
    ADC_InjectedChannelConfig(ADC3, CURR_C_CHANNEL, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC3, VSENSE_C_CHANNEL, 2, ADC_SampleTime_15Cycles);

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

    palSetPad(EN_GATE_GPIO, EN_GATE_PIN);
}

/* Updates the controller state machine. Called as an interrupt handler on new ADC sample. */
void controller_update(void)
{
    WWDG_SetCounter(100);
    adc1_1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    adc2_1 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    adc3_1 = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1);
    adc1_2 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    adc2_2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
    adc3_2 = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_2);
    int adc1_3 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
    int adc2_3 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3);
    motor_state.v_bus = adc1_3 * (V_REG / 4095.0) * (VBUS_R1 + VBUS_R2) / VBUS_R2;
    /*temp = (1.0 / ((logf(((4095.0 * 10000.0) / adc2_3 - 10000.0) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);*/
    if (fault == OVERCURRENT || !(CHECK_CURRENT(adc1_1) && CHECK_CURRENT(adc2_1) && CHECK_CURRENT(adc3_1)))
    {
        SET_DUTY(0, 0, 0);
        palClearPad(EN_GATE_GPIO, EN_GATE_PIN);
        fault = OVERCURRENT;
        return;
    }
    motor_state.i_a = ((int16_t)adc1_1 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    motor_state.i_b = ((int16_t)adc2_1 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    motor_state.i_c = ((int16_t)adc3_1 - 2048) * (V_REG / 4095.0) / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);
    // Check for ADC measurements that happened when the low side PWM was too short
    if (motor_state.v_a_norm > 0.95)
    {
        motor_state.i_a = -(motor_state.i_b + motor_state.i_c);
    }
    else if (motor_state.v_b_norm > 0.95)
    {
        motor_state.i_b = -(motor_state.i_a + motor_state.i_c);
    }
    else if (motor_state.v_c_norm > 0.95)
    {
        motor_state.i_c = -(motor_state.i_a + motor_state.i_b);
    }
    // If all are good, use all the balanced current assumption to use all three channels to improve each channel's measurement
    else if (motor_state.v_a_norm <= 0.95 && motor_state.v_b_norm <= 0.95 && motor_state.v_c_norm <= 0.95)
    {
        motor_state.i_a = motor_state.i_a / 2.0 - (motor_state.i_b + motor_state.i_c) / 2.0;
        motor_state.i_b = motor_state.i_b / 2.0 - (motor_state.i_a + motor_state.i_c) / 2.0;
        motor_state.i_c = motor_state.i_c / 2.0 - (motor_state.i_a + motor_state.i_b) / 2.0;
    }
    const float dt = 1.0 / config->pwmFrequency;
    switch(state)
    {
        case STOPPED:
            motor_state.edeg = encoder_get_angle();
            SET_DUTY(0, 0, 0);
            break;
        case RUNNING:
            motor_state.edeg = encoder_get_angle();
            transforms_clarke(motor_state.i_a, motor_state.i_b, motor_state.i_c, &motor_state.i_alpha, &motor_state.i_beta);
            transforms_park(motor_state.i_alpha, motor_state.i_beta, motor_state.edeg, &motor_state.i_d, &motor_state.i_q);
            float i_d_set = 0.0;
            float i_q_set = 3.0;
            float i_d_err = i_d_set - motor_state.i_d;
            float i_q_err = i_q_set - motor_state.i_q;
            motor_state.integral_d += i_d_err * config->currentKi * dt;
            motor_state.integral_q += i_q_err * config->currentKi * dt;
            utils_saturate_vector_2d((float*)&motor_state.integral_d, (float*)&motor_state.integral_q,
                    (2.0 / 3.0) * config->maxDuty * SQRT3_BY_2 * motor_state.v_bus);
            // TODO: Add back-EMF as feed forward term
            motor_state.v_d = motor_state.integral_d + i_d_err * config->currentKp;
            motor_state.v_q = motor_state.integral_q + i_q_err * config->currentKp;
            motor_state.v_d_norm = motor_state.v_d / ((2.0 / 3.0) * motor_state.v_bus);
            motor_state.v_q_norm = motor_state.v_q / ((2.0 / 3.0) * motor_state.v_bus);
            utils_saturate_vector_2d((float*)&motor_state.v_d_norm, (float*)&motor_state.v_q_norm,
                    SQRT3_BY_2 * config->maxDuty);
            transforms_inverse_park(motor_state.v_d_norm, motor_state.v_q_norm, motor_state.edeg, &motor_state.v_alpha_norm, &motor_state.v_beta_norm);
            transforms_inverse_clarke(motor_state.v_alpha_norm, motor_state.v_beta_norm, &motor_state.v_a_norm, &motor_state.v_b_norm, &motor_state.v_c_norm);
            apply_zsm(&motor_state.v_a_norm, &motor_state.v_b_norm, &motor_state.v_c_norm);
            SET_DUTY(motor_state.v_a_norm, motor_state.v_b_norm, motor_state.v_c_norm);
            break;
        case ZEROING:
            if (e++ < 4000)
            {
                float alpha, beta;
                transforms_inverse_park(0.2, 0, 0, &alpha, &beta);
                transforms_inverse_clarke(alpha, beta, &a, &b, &c);
                apply_zsm(&a, &b, &c);
                SET_DUTY(a, b, c);
                motor_state.edeg = encoder_get_raw_angle();
                config->encoderZero = motor_state.edeg; 
            }
            else
            {
                e = 0;
                state = RUNNING;
            }
            break;
    }

}

void controller_set_duty(float duty)
{
    if (duty > 1.0)
    {
        duty = 1.0;
    }
    else if (duty < -1.0)
    {
        duty = -1.0;
    }
    commandDutyCycle = duty;
}

void controller_set_running(bool enable)
{
    if (enable)
    {
        state = RUNNING;
    }
    else
    {
        state = STOPPED;
    }
}

void controller_print(void)
{
    USB_PRINT("%f, %f, %f, %f, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f\n", motor_state.edeg, motor_state.v_a_norm, motor_state.v_b_norm, motor_state.v_c_norm, adc1_1, adc2_1, adc3_1, motor_state.i_a, motor_state.i_b, motor_state.i_c, motor_state.i_d, motor_state.i_q, motor_state.v_d, motor_state.v_q);
}

static void apply_zsm(volatile float *a, volatile float *b, volatile float *c)
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
