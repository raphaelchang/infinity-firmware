#include "as5x47.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"

static void spicb(SPIDriver *spip);
static bool inTransaction = false;
static uint8_t rx[2];
static uint8_t tx[2];
static uint16_t lastRawPosition = 0;

static const SPIConfig encoderSPI =
{
    spicb,
    GPIOA,
    GPIOA_LRCK,
    0
};

CH_IRQ_HANDLER(TIM4_IRQHandler) {
    chSysLockFromISR();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
		// as5x47_update_raw_position();

        palClearPad(GPIOA, GPIOA_LRCK);
		// Clear the IT pending bit
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
    chSysUnlockFromISR();
}

void as5x47_init(void)
{
	palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);     /* SCK. */
	palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);     /* MISO.*/
	palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);     /* MOSI.*/
	palSetPadMode(GPIOA, GPIOA_LRCK, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);
	palSetPad(GPIOA, GPIOA_LRCK);
    spiStart(&SPID1, &encoderSPI);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Time Base configuration
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / 20000) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	nvicEnableVector(TIM4_IRQn, 6);

	// Enable overflow interrupt
	// TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	// Enable timers
	TIM_Cmd(TIM4, ENABLE);

}

float as5x47_get_angle(void)
{
	as5x47_update_raw_position();
	float raw = as5x47_get_raw_position() % 8192;
	return 360.0 * raw / 8192.0;
}

void as5x47_update_raw_position(void)
{
	spiSelectI(&SPID1);
	tx[0] = 0x3F | (1 << 6);
	tx[1] = 0xFF;
	spiStartSendI(&SPID1, 2, tx);
	inTransaction = true;
}

uint16_t as5x47_get_raw_position(void)
{
	return lastRawPosition;
}

static void spicb(SPIDriver *spip) {
	chSysLockFromISR();
	spiUnselectI(spip);
	if (inTransaction)
	{
		int i = 0;
		for(i = 0; i < 20; i++)
		{
			__asm__("NOP");
		}
		spiSelectI(spip);
		tx[0] = 0;
		tx[1] = 0;
		spiStartExchangeI(spip, 2, tx, rx);
		inTransaction = false;
	}
	else
	{
		lastRawPosition = (uint16_t)(((rx[0] << 8) | rx[1]) & 0x3FFF);
	}
	chSysUnlockFromISR();
}