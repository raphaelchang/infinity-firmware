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
#include <math.h>
#include "utils.h"
#include "console.h"
#include "scope.h"

#define SYSTEM_CORE_CLOCK       168000000
#define DEAD_TIME_CYCLES        60

#define SET_DUTY(duty1, duty2, duty3) \
        TIM1->CR1 |= TIM_CR1_UDIS; \
        TIM1->CCR1 = (int)(duty3 * TIM1->ARR); \
        TIM1->CCR2 = (int)(duty2 * TIM1->ARR); \
        TIM1->CCR3 = (int)(duty1 * TIM1->ARR); \
        TIM1->CCR4 = TIM1->ARR - 2; \
        TIM1->CR1 &= ~TIM_CR1_UDIS;

#define SET_PWM_FREQ(freq) \
        TIM1->CR1 |= TIM_CR1_UDIS; \
        if (freq < 100) \
        { \
            TIM1->PSC = 31; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / (freq * 32); \
        } \
        else if (freq < 200) \
        { \
            TIM1->PSC = 15; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / (freq * 16); \
        } \
        else if (freq < 400) \
        { \
            TIM1->PSC = 7; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / (freq * 8); \
        } \
        else if (freq < 700) \
        { \
            TIM1->PSC = 3; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / (freq * 4); \
        } \
        else if (freq < 1300) \
        { \
            TIM1->PSC = 1; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / (freq * 2); \
        } \
        else \
        { \
            TIM1->PSC = 0; \
            TIM1->ARR = SYSTEM_CORE_CLOCK / 2 / freq; \
        } \
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
static volatile ControllerState state = STOPPED;
static volatile ControllerFault fault = NO_FAULT;
static volatile ControllerMode mode = NONE;
static volatile uint16_t ADC_Value[3];

static volatile motor_state_t motor_state;
static volatile int adc1_1;
static volatile int adc2_1;
static volatile int adc3_1;
static volatile int adc1_2;
static volatile int adc2_2;
static volatile int adc3_2;
static volatile float temperature;
static volatile int e = 0;
static volatile float a, b, c;
static volatile float commandDutyCycle = 0.0;
static volatile float commandCurrentQ = 0.0;
static volatile float commandCurrentD = 0.0;
static volatile float commandSpeed = 0.0;
static volatile bool overrideAngle = false;
static volatile float angleToOverride;
static volatile bool pwmEnabled = false;
static volatile float pllAngle;
static volatile float pllSpeed;
static volatile float speedIntegral = 0.0;
static volatile float speedPrevError = 0.0;

static void apply_zsm(volatile float *a, volatile float *b, volatile float *c);
static void pll_run(float angle, float dt, volatile float *pll_angle,
	volatile float *pll_speed);
static void disable_pwm(void);
static void enable_pwm(void);
static void play_startup_tone(void);

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
    overrideAngle = false;
    pwmEnabled = false;
    pllAngle = 0.0;
    pllSpeed = 0.0;

    utils_sys_lock_cnt();
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

    disable_pwm();

    utils_sys_unlock_cnt();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    WWDG_SetPrescaler(WWDG_Prescaler_1);
    WWDG_SetWindowValue(255);
    WWDG_Enable(100);

    palSetPad(EN_GATE_GPIO, EN_GATE_PIN);

    play_startup_tone();

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
    temperature = adc2_3;
    motor_state.v_bus = adc1_3 * (V_REG / 4095.0) * (VBUS_R1 + VBUS_R2) / VBUS_R2;
    if (fault == OVERCURRENT || !(CHECK_CURRENT(adc1_1) && CHECK_CURRENT(adc2_1) && CHECK_CURRENT(adc3_1)))
    {
        disable_pwm();
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
    transforms_clarke(motor_state.i_a, motor_state.i_b, motor_state.i_c, &motor_state.i_alpha, &motor_state.i_beta);
    transforms_park(motor_state.i_alpha, motor_state.i_beta, motor_state.edeg, &motor_state.i_d, &motor_state.i_q);
    const float dt = 1.0 / config->pwmFrequency;
    switch(state)
    {
        case STOPPED:
            motor_state.edeg = encoder_get_angle();
            SET_DUTY(0, 0, 0);
            break;
        case RUNNING:
            motor_state.edeg = encoder_get_angle();
            if (overrideAngle)
            {
                motor_state.edeg = angleToOverride;
            }
            if (mode == SPEED)
            {
                float p_term;
                float d_term;

                // Too low RPM set. Reset state and return.
                /*if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {*/
                    /*i_term = 0.0;*/
                    /*prev_error = 0;*/
                    /*return;*/
                /*}*/

                // Compensation for supply voltage variations
                float scale = 1.0 / motor_state.v_bus;

                // Compute error
                float error = commandSpeed - controller_get_erpm();

                // Compute parameters
                p_term = error * config->speedKp * scale;
                speedIntegral += error * (config->speedKi * dt) * scale;
                d_term = (error - speedPrevError) * (config->speedKd / dt) * scale;

                // I-term wind-up protection
                if (speedIntegral > 1.0)
                {
                    speedIntegral = 1.0;
                }
                else if (speedIntegral < -1.0)
                {
                    speedIntegral = -1.0;
                }

                speedPrevError = error;

                // Calculate output
                float output = p_term + speedIntegral + d_term;
                if (output > 1.0)
                {
                    output = 1.0;
                }
                else if (output < -1.0)
                {
                    output = -1.0;
                }

                commandCurrentQ = output * config->maxCurrent;
            }
            else
            {
                speedIntegral = 0.0;
                speedPrevError = 0.0;
            }
            float i_d_set = commandCurrentD;
            float i_q_set = commandCurrentQ;
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
        case TONE:
            SET_DUTY(0.005, 0, 0);
            break;
    }
    pll_run(motor_state.edeg, dt, &pllAngle, &pllSpeed);
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
    state = RUNNING;
    mode = DUTY_CYCLE;
    if (!pwmEnabled)
        enable_pwm();
}

void controller_set_current(float current)
{
    if (current > config->maxCurrent)
    {
        current = config->maxCurrent;
    }
    else if (current < -config->maxCurrent)
    {
        current = -config->maxCurrent;
    }
    commandCurrentQ = current;
    commandCurrentD = 0.0;
    state = RUNNING;
    mode = CURRENT;
    if (!pwmEnabled)
        enable_pwm();
}

void controller_set_speed(float speed)
{
    commandSpeed = speed;
    state = RUNNING;
    mode = SPEED;
    if (!pwmEnabled)
        enable_pwm();
}

void controller_disable(void)
{
    state = STOPPED;
    mode = NONE;
    motor_state.integral_d = 0.0;
    motor_state.integral_q = 0.0;
    commandCurrentQ = 0.0;
    commandCurrentD = 0.0;
    commandSpeed = 0.0;
    commandDutyCycle = 0.0;
    if (pwmEnabled)
        disable_pwm();
}

float controller_get_bus_voltage(void)
{
    return motor_state.v_bus;
}

float controller_get_temperature(void)
{
    return (1.0 / ((logf(((4095.0 * 10000.0) / temperature - 10000.0) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);
}

bool controller_encoder_zero(float current, float *zero, bool *inverted)
{
    overrideAngle = true;
    angleToOverride = 0.0;
    commandCurrentQ = 0.0;
    commandCurrentD = current;
    state = RUNNING;
    if (!pwmEnabled)
        enable_pwm();
    chThdSleepMilliseconds(800);
    *zero = encoder_get_raw_angle();
    commandCurrentD = current / 2.0;
    float i = 0;
    float deg = *zero;
    int incCount = 0;
    for (i = 1.0; i <= 180.0; i++)
    {
        angleToOverride = i;
        chThdSleepMilliseconds(1);
        float temp = encoder_get_raw_angle();
        if (temp > deg)
        {
            incCount++;
        }
        else if (temp < deg)
        {
            incCount--;
        }
        deg = temp;
    }
    if (incCount == 0)
    {
        return false;
    }
    *inverted = incCount < 0;
    commandCurrentQ = commandCurrentD = 0.0;
    overrideAngle = false;
    controller_disable();
    return true;
}

float controller_measure_resistance(float current, uint16_t samples)
{
    overrideAngle = true;
    angleToOverride = 0.0;
    commandCurrentQ = current;
    commandCurrentD = 0.0;
    const float pwmFreqOld = config->pwmFrequency;
    const float currentKpOld = config->currentKp;
    const float currentKiOld = config->currentKi;
    config->pwmFrequency = 10000.0;
    config->currentKp = 0.01;
    config->currentKi = 10.0;
    SET_PWM_FREQ(config->pwmFrequency);
    state = RUNNING;
    if (!pwmEnabled)
        enable_pwm();
    chThdSleepMilliseconds(1000);

    float current_sum = 0;
    float voltage_sum = 0;
    uint16_t i = 0;
    for (i = 0; i < samples; i++)
    {
        const volatile float vd = motor_state.v_d;
        const volatile float vq = motor_state.v_q;
        const volatile float id = motor_state.i_d;
        const volatile float iq = motor_state.i_q;

        current_sum += sqrtf(id * id + iq * iq);
        voltage_sum += sqrtf(vd * vd + vq * vq);
        chThdSleepMilliseconds(1);
    }
    commandCurrentQ = commandCurrentD = 0.0;
    overrideAngle = false;
    controller_disable();
    config->pwmFrequency = pwmFreqOld;
    config->currentKp = currentKpOld;
    config->currentKi = currentKiOld;
    SET_PWM_FREQ(config->pwmFrequency);
    float current_avg = current_sum / samples;
    float voltage_avg = voltage_sum / samples;
    return (voltage_avg / current_avg) * (2.0 / 3.0);
}

float controller_get_erpm(void)
{
    return pllSpeed / ((2.0 * M_PI) / 60.0);
}

float controller_get_current_q(void)
{
    return motor_state.i_q;
}

float controller_get_current_d(void)
{
    return motor_state.i_d;
}

void controller_print(void)
{
    /*USB_PRINT("%f, %f, %f, %f, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f\n", motor_state.edeg, motor_state.v_a_norm, motor_state.v_b_norm, motor_state.v_c_norm, adc1_1, adc2_1, adc3_1, motor_state.i_a, motor_state.i_b, motor_state.i_c, motor_state.i_d, motor_state.i_q, motor_state.v_d, motor_state.v_q);*/
    USB_PRINT("%f, %f, %f, %d\n", controller_get_erpm(), commandCurrentQ, commandSpeed, mode);
}

static void pll_run(float angle, float dt, volatile float *pll_angle,
	volatile float *pll_speed) {
    angle *= M_PI / 180.0;
    float delta_theta = angle - *pll_angle;
    utils_norm_angle_rad(&delta_theta);
    *pll_angle += (*pll_speed + config->pllKp * delta_theta) * dt;
    utils_norm_angle_rad((float*)pll_angle);
    *pll_speed += config->pllKi * delta_theta * dt;
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

static void disable_pwm(void)
{
    SET_DUTY(0, 0, 0);

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    pwmEnabled = false;
}

static void enable_pwm(void)
{
    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    pwmEnabled = true;
}

ControllerFault controller_get_fault(void)
{
    return fault;
}

ControllerState controller_get_state(void)
{
    return state;
}

float controller_get_command_current(void)
{
    return commandCurrentQ;
}

typedef enum
{
    NOTE_C0 = 0,
    NOTE_CS0,
    NOTE_D0,
    NOTE_DS0,
    NOTE_E0,
    NOTE_F0,
    NOTE_FS0,
    NOTE_G0,
    NOTE_GS0,
    NOTE_A0,
    NOTE_AS0,
    NOTE_B0,
    NOTE_C1,
    NOTE_CS1,
    NOTE_D1,
    NOTE_DS1,
    NOTE_E1,
    NOTE_F1,
    NOTE_FS1,
    NOTE_G1,
    NOTE_GS1,
    NOTE_A1,
    NOTE_AS1,
    NOTE_B1,
    NOTE_C2,
    NOTE_CS2,
    NOTE_D2,
    NOTE_DS2,
    NOTE_E2,
    NOTE_F2,
    NOTE_FS2,
    NOTE_G2,
    NOTE_GS2,
    NOTE_A2,
    NOTE_AS2,
    NOTE_B2,
    NOTE_C3,
    NOTE_CS3,
    NOTE_D3,
    NOTE_DS3,
    NOTE_E3,
    NOTE_F3,
    NOTE_FS3,
    NOTE_G3,
    NOTE_GS3,
    NOTE_A3,
    NOTE_AS3,
    NOTE_B3,
    NOTE_C4,
    NOTE_CS4,
    NOTE_D4,
    NOTE_DS4,
    NOTE_E4,
    NOTE_F4,
    NOTE_FS4,
    NOTE_G4,
    NOTE_GS4,
    NOTE_A4,
    NOTE_AS4,
    NOTE_B4,
    NOTE_C5,
    NOTE_CS5,
    NOTE_D5,
    NOTE_DS5,
    NOTE_E5,
    NOTE_F5,
    NOTE_FS5,
    NOTE_G5,
    NOTE_GS5,
    NOTE_A5,
    NOTE_AS5,
    NOTE_B5,
    NOTE_C6,
    NOTE_CS6,
    NOTE_D6,
    NOTE_DS6,
    NOTE_E6,
    NOTE_F6,
    NOTE_FS6,
    NOTE_G6,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C7,
    NOTE_CS7,
    NOTE_D7,
    NOTE_DS7,
    NOTE_E7,
    NOTE_F7,
    NOTE_FS7,
    NOTE_G7,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
    NOTE_C8,
    NOTE_CS8,
    NOTE_D8,
    NOTE_DS8,
    NOTE_E8,
    NOTE_F8,
    NOTE_FS8,
    NOTE_G8,
    NOTE_GS8,
    NOTE_A8,
    NOTE_AS8,
    NOTE_B8,
    NOTE_C9,
    NOTE_CS9,
    NOTE_D9,
    NOTE_DS9,
    NOTE_E9,
    NOTE_F9,
    NOTE_FS9,
    NOTE_G9,
    NOTE_GS9,
    NOTE_A9,
    NOTE_AS9,
    NOTE_B9,
} note_t;
static float midi[120] = {16.3515978313, 17.3239144361, 18.3540479948, 19.4454364826, 20.6017223071, 21.8267644646, 23.1246514195, 24.4997147489, 25.9565435987, 27.5, 29.1352350949, 30.8677063285, 32.7031956626, 34.6478288721, 36.7080959897, 38.8908729653, 41.2034446141, 43.6535289291, 46.249302839, 48.9994294977, 51.9130871975, 55.0, 58.2704701898, 61.735412657, 65.4063913251, 69.2956577442, 73.4161919794, 77.7817459305, 82.4068892282, 87.3070578583, 92.4986056779, 97.9988589954, 103.826174395, 110.0, 116.54094038, 123.470825314, 130.81278265, 138.591315488, 146.832383959, 155.563491861, 164.813778456, 174.614115717, 184.997211356, 195.997717991, 207.65234879, 220.0, 233.081880759, 246.941650628, 261.625565301, 277.182630977, 293.664767917, 311.126983722, 329.627556913, 349.228231433, 369.994422712, 391.995435982, 415.30469758, 440.0, 466.163761518, 493.883301256, 523.251130601, 554.365261954, 587.329535835, 622.253967444, 659.255113826, 698.456462866, 739.988845423, 783.990871963, 830.60939516, 880.0, 932.327523036, 987.766602512, 1046.5022612, 1108.73052391, 1174.65907167, 1244.50793489, 1318.51022765, 1396.91292573, 1479.97769085, 1567.98174393, 1661.21879032, 1760.0, 1864.65504607, 1975.53320502, 2093.0045224, 2217.46104781, 2349.31814334, 2489.01586978, 2637.0204553, 2793.82585146, 2959.95538169, 3135.96348785, 3322.43758064, 3520.0, 3729.31009214, 3951.06641005, 4186.00904481, 4434.92209563, 4698.63628668, 4978.03173955, 5274.04091061, 5587.65170293, 5919.91076339, 6271.92697571, 6644.87516128, 7040.0, 7458.62018429, 7902.1328201, 8372.01808962, 8869.84419126, 9397.27257336, 9956.06347911, 10548.0818212, 11175.3034059, 11839.8215268, 12543.8539514, 13289.7503226, 14080.0, 14917.2403686, 15804.2656402};
#define PLAY_NOTE(note, duration) \
    SET_PWM_FREQ(midi[note]); \
    chThdSleepMilliseconds(duration);
#define PLAY_REST(duration) \
    state = STOPPED; \
    chThdSleepMilliseconds(duration); \
    state = TONE;

static void play_startup_tone(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    state = TONE;
    if (!pwmEnabled)
        enable_pwm();
    PLAY_NOTE(NOTE_B4, 300);
    PLAY_REST(100);
    PLAY_NOTE(NOTE_B4, 300);
    PLAY_REST(100);
    PLAY_NOTE(NOTE_B4, 300);
    PLAY_REST(500);
    PLAY_NOTE(NOTE_B4, 150);
    PLAY_NOTE(NOTE_D5, 150);
    PLAY_NOTE(NOTE_E5, 150);
    /*for (uint8_t i = NOTE_C2; i <= NOTE_B9; i++)*/
    /*{*/
        /*PLAY_NOTE(i, 100);*/
    /*}*/
    /*PLAY_REST(200);*/
    /*PLAY_NOTE(NOTE_C4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_C4, 200);*/
    /*PLAY_NOTE(NOTE_G4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 200);*/
    /*PLAY_NOTE(NOTE_A4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_A4, 200);*/
    /*PLAY_NOTE(NOTE_G4, 400);*/
    /*PLAY_NOTE(NOTE_F4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_F4, 200);*/
    /*PLAY_NOTE(NOTE_E4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_E4, 200);*/
    /*PLAY_NOTE(NOTE_D4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_D4, 200);*/
    /*PLAY_NOTE(NOTE_C4, 400);*/
    /*PLAY_NOTE(NOTE_G4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 200);*/
    /*PLAY_NOTE(NOTE_F4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_F4, 200);*/
    /*PLAY_NOTE(NOTE_E4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_E4, 200);*/
    /*PLAY_NOTE(NOTE_D4, 400);*/
    /*PLAY_NOTE(NOTE_G4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 200);*/
    /*PLAY_NOTE(NOTE_F4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_F4, 200);*/
    /*PLAY_NOTE(NOTE_E4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_E4, 200);*/
    /*PLAY_NOTE(NOTE_D4, 400);*/
    /*PLAY_NOTE(NOTE_C4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_C4, 200);*/
    /*PLAY_NOTE(NOTE_G4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 200);*/
    /*PLAY_NOTE(NOTE_A4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_A4, 200);*/
    /*PLAY_NOTE(NOTE_G4, 400);*/
    /*PLAY_NOTE(NOTE_F4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_F4, 200);*/
    /*PLAY_NOTE(NOTE_E4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_E4, 200);*/
    /*PLAY_NOTE(NOTE_D4, 190);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_D4, 200);*/
    /*PLAY_NOTE(NOTE_C4, 400);*/

    /*PLAY_REST(200);*/
    /*PLAY_NOTE(NOTE_G4, 290);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 100);*/
    /*PLAY_NOTE(NOTE_A4, 400);*/
    /*PLAY_NOTE(NOTE_G4, 400);*/
    /*PLAY_NOTE(NOTE_C5, 400);*/
    /*PLAY_NOTE(NOTE_B4, 800);*/

    /*PLAY_NOTE(NOTE_G4, 290);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 100);*/
    /*PLAY_NOTE(NOTE_A4, 400);*/
    /*PLAY_NOTE(NOTE_G4, 400);*/
    /*PLAY_NOTE(NOTE_D5, 400);*/
    /*PLAY_NOTE(NOTE_C5, 800);*/

    /*PLAY_NOTE(NOTE_G4, 290);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_G4, 100);*/
    /*PLAY_NOTE(NOTE_G5, 400);*/
    /*PLAY_NOTE(NOTE_E5, 400);*/
    /*PLAY_NOTE(NOTE_C5, 400);*/
    /*PLAY_NOTE(NOTE_B4, 400);*/
    /*PLAY_NOTE(NOTE_A4, 800);*/

    /*PLAY_NOTE(NOTE_F5, 290);*/
    /*PLAY_REST(10);*/
    /*PLAY_NOTE(NOTE_F5, 100);*/
    /*PLAY_NOTE(NOTE_E5, 400);*/
    /*PLAY_NOTE(NOTE_C5, 400);*/
    /*PLAY_NOTE(NOTE_D5, 400);*/
    /*PLAY_NOTE(NOTE_C5, 800);*/

    state = STOPPED;
    disable_pwm();
    SET_PWM_FREQ(config->pwmFrequency);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
}
