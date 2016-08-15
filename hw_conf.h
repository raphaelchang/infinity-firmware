#ifndef _HW_CONF_H_
#define _HW_CONF_H_

#include "ch.h"
#include "hal.h"

// PWM
#define PWM_A_HIGH_GPIO GPIOA
#define PWM_A_HIGH_PIN 10
#define PWM_B_HIGH_GPIO GPIOA
#define PWM_B_HIGH_PIN 9
#define PWM_C_HIGH_GPIO GPIOA
#define PWM_C_HIGH_PIN 8
#define PWM_A_LOW_GPIO GPIOB
#define PWM_A_LOW_PIN 15
#define PWM_B_LOW_GPIO GPIOB
#define PWM_B_LOW_PIN 14
#define PWM_C_LOW_GPIO GPIOB
#define PWM_C_LOW_PIN 13

// Gate Power
#define EN_GATE_GPIO GPIOA
#define EN_GATE_PIN 3

// ADC
#define CURR_A_GPIO GPIOC
#define CURR_B_GPIO GPIOC
#define CURR_C_GPIO GPIOA
#define CURR_A_PIN 1
#define CURR_B_PIN 3
#define CURR_C_PIN 1
#define VSENSE_A_GPIO GPIOC
#define VSENSE_B_GPIO GPIOA
#define VSENSE_C_GPIO GPIOA
#define VSENSE_A_PIN 2
#define VSENSE_B_PIN 0
#define VSENSE_C_PIN 2
#define VBUS_GPIO GPIOB
#define VBUS_PIN 1
#define NTC_GPIO GPIOC
#define NTC_PIN 0
#define CURR_A_CHANNEL ADC_Channel_11
#define CURR_B_CHANNEL ADC_Channel_13
#define CURR_C_CHANNEL ADC_Channel_1
#define VSENSE_A_CHANNEL ADC_Channel_12
#define VSENSE_B_CHANNEL ADC_Channel_0
#define VSENSE_C_CHANNEL ADC_Channel_2
#define VBUS_CHANNEL ADC_Channel_9
#define NTC_CHANNEL ADC_Channel_10

// Encoder
#define ENCODER_SCK_GPIO GPIOA
#define ENCODER_MISO_GPIO GPIOA
#define ENCODER_MOSI_GPIO GPIOA
#define ENCODER_NSS_GPIO GPIOA
#define ENCODER_SCK_PIN 5
#define ENCODER_MISO_PIN 6
#define ENCODER_MOSI_PIN 7
#define ENCODER_NSS_PIN GPIOA_LRCK

// WS2812B
#define WS2812B_GPIO GPIOC
#define WS2812B_PIN 9

#define CURRENT_SENSE_RES 0.001
#define CURRENT_AMP_GAIN 20
#define V_REG 3.3
#define VBUS_R1 200000.0
#define VBUS_R2 10000.0

// I2C
#define I2C_DEV I2CD1
#define I2C_GPIO GPIOB
#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 7

#endif
