#include "led_rgb.h"
#include "ch.h"
#include "hal.h"
#include "hw_conf.h"
#if defined(INFINITY_V4_0) || defined(INFINITY_V3_0)
#include "ws2812b.h"
#else
static PWMConfig pwmcfg = {
    512000,
    256,
    NULL,
    {
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_ACTIVE_LOW, NULL},
        {PWM_OUTPUT_ACTIVE_LOW, NULL},
        {PWM_OUTPUT_ACTIVE_LOW, NULL}
    },
    0,
    0
};
#endif

void led_rgb_init(void)
{
#if defined(INFINITY_V4_0) || defined(INFINITY_V3_0)
    ws2812b_init();
#else
    pwmStart(&LED_PWM_DEV, &pwmcfg);
    pwmEnableChannel(&LED_PWM_DEV, LED_R_CHANNEL, 0);
    pwmEnableChannel(&LED_PWM_DEV, LED_G_CHANNEL, 0);
    pwmEnableChannel(&LED_PWM_DEV, LED_B_CHANNEL, 0);
#endif
}
void led_rgb_set(uint32_t color)
{
#if defined(INFINITY_V4_0) || defined(INFINITY_V3_0)
    ws2812b_set_all(color);
#else
    pwmEnableChannel(&LED_PWM_DEV, LED_B_CHANNEL, color & 0xFF);
    color = color >> 8;
    pwmEnableChannel(&LED_PWM_DEV, LED_G_CHANNEL, color & 0xFF);
    color = color >> 8;
    pwmEnableChannel(&LED_PWM_DEV, LED_R_CHANNEL, color & 0xFF);
#endif
}
