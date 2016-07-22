#ifndef _HW_CONF_H_
#define _HW_CONF_H_

#include "ch.h"
#include "hal.h"

// PWM
#define PWM_A_HIGH_GPIO GPIOA
#define PWM_A_HIGH_PIN 8
#define PWM_B_HIGH_GPIO GPIOA
#define PWM_B_HIGH_PIN 9
#define PWM_C_HIGH_GPIO GPIOA
#define PWM_C_HIGH_PIN 10
#define PWM_A_LOW_GPIO GPIOB
#define PWM_A_LOW_PIN 13
#define PWM_B_LOW_GPIO GPIOB
#define PWM_B_LOW_PIN 14
#define PWM_C_LOW_GPIO GPIOB
#define PWM_C_LOW_PIN 15

// ADC
#define CURR_A_GPIO GPIOA
#define CURR_B_GPIO GPIOA
#define CURR_C_GPIO GPIOA
#define CURR_A_PIN 0
#define CURR_B_PIN 1
#define CURR_C_PIN 2
#define CURR_A_CHANNEL ADC_Channel_0
#define CURR_B_CHANNEL ADC_Channel_1
#define CURR_C_CHANNEL ADC_Channel_2

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
#define WS2812B_GPIO GPIOB
#define WS2812B_PIN 0

#define CURRENT_SENSE_RES 0.001
#define CURRENT_AMP_GAIN 20
#define V_REG 3.3

// I2C
#define I2C_DEV I2CD1
#define I2C_GPIO GPIOB
#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 7

#endif
