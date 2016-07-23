#include "comm_nunchuk.h"

#include "ch.h"
#include "hal.h"
#include "hw_conf.h"
#include "comm_usb.h"
#include <string.h>

static const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static uint8_t rxbuf[10];
static uint8_t txbuf[10];
static msg_t status = MSG_OK;
static i2caddr_t addr = 0x52;
static systime_t tmo = MS2ST(5);

void comm_nunchuk_init(void)
{
    i2cAcquireBus(&I2C_DEV); 

    palSetPadMode(I2C_GPIO, I2C_SCL_PIN,
            PAL_MODE_ALTERNATE(4) |
            PAL_STM32_OTYPE_OPENDRAIN |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
            PAL_MODE_ALTERNATE(4) |
            PAL_STM32_OTYPE_OPENDRAIN |
            PAL_STM32_OSPEED_MID1);

    i2cStart(&I2C_DEV, &i2ccfg);
    i2cReleaseBus(&I2C_DEV);
}

void comm_nunchuk_update(void)
{
    bool is_ok = true;

    txbuf[0] = 0xF0;
    txbuf[1] = 0x55;
    i2cAcquireBus(&I2C_DEV);
    status = i2cMasterTransmitTimeout(&I2C_DEV, addr, txbuf, 2, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);
    is_ok = status == MSG_OK;

    USB_PRINT("Update nunchuk 0 %d\n", i2cGetErrors(&I2C_DEV));
    if (is_ok) {
        txbuf[0] = 0xFB;
        txbuf[1] = 0x00;
        i2cAcquireBus(&I2C_DEV);
        status = i2cMasterTransmitTimeout(&I2C_DEV, addr, txbuf, 2, rxbuf, 0, tmo);
        i2cReleaseBus(&I2C_DEV);
        is_ok = status == MSG_OK;
    }
    USB_PRINT("Update nunchuk 1 %d\n", i2cGetErrors(&I2C_DEV));

    if (is_ok) {
        txbuf[0] = 0x00;
        i2cAcquireBus(&I2C_DEV);
        status = i2cMasterTransmitTimeout(&I2C_DEV, addr, txbuf, 1, rxbuf, 0, tmo);
        i2cReleaseBus(&I2C_DEV);
        is_ok = status == MSG_OK;
    }
    USB_PRINT("Update nunchuk 2 %d\n", i2cGetErrors(&I2C_DEV));

    if (is_ok) {
        chThdSleepMilliseconds(3);

        i2cAcquireBus(&I2C_DEV);
        status = i2cMasterReceiveTimeout(&I2C_DEV, addr, rxbuf, 6, tmo);
        i2cReleaseBus(&I2C_DEV);
        is_ok = status == MSG_OK;
    }
    USB_PRINT("Update nunchuk 3 %d\n", is_ok);

    if (is_ok) {
        static uint8_t last_buffer[6];
        int same = 1;

        for (int i = 0;i < 6;i++) {
            if (last_buffer[i] != rxbuf[i]) {
                same = 0;
            }
        }

        memcpy(last_buffer, rxbuf, 6);

        if (!same) {
            int x = rxbuf[0];
            int y = rxbuf[1];
            int acc_x = (rxbuf[2] << 2) | ((rxbuf[5] >> 2) & 3);
            int acc_y = (rxbuf[3] << 2) | ((rxbuf[5] >> 4) & 3);
            int acc_z = (rxbuf[4] << 2) | ((rxbuf[5] >> 6) & 3);
            bool btn_z = !((rxbuf[5] >> 0) & 1);
            bool btn_c = !((rxbuf[5] >> 1) & 1);
            USB_PRINT("%d %d %d %d %d %d %d\n", x, y, acc_x, acc_y, acc_z, btn_z, btn_c);
            if (btn_c)
            {
                controller_set_running(true);
                controller_set_duty((y - 128) / 128.0 / 5.0);
            }
            else
            {
                controller_set_running(false);
            }
        }
    } else {
        i2cAcquireBus(&I2C_DEV);

        palSetPadMode(I2C_GPIO, I2C_SCL_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1);

        palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1);

        palSetPad(I2C_GPIO, I2C_SCL_PIN);
        palSetPad(I2C_GPIO, I2C_SDA_PIN);

        chThdSleep(1);

        for(int i = 0; i < 16; i++) {
            palClearPad(I2C_GPIO, I2C_SCL_PIN);
            chThdSleep(1);
            palSetPad(I2C_GPIO, I2C_SCL_PIN);
            chThdSleep(1);
        }

        palClearPad(I2C_GPIO, I2C_SDA_PIN);
        chThdSleep(1);
        palClearPad(I2C_GPIO, I2C_SCL_PIN);
        chThdSleep(1);
        palSetPad(I2C_GPIO, I2C_SCL_PIN);
        chThdSleep(1);
        palSetPad(I2C_GPIO, I2C_SDA_PIN);

        palSetPadMode(I2C_GPIO, I2C_SCL_PIN,
                PAL_MODE_ALTERNATE(4) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1);

        palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
                PAL_MODE_ALTERNATE(4) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1);

        I2C_DEV.state = I2C_STOP;
        i2cStart(&I2C_DEV, &i2ccfg);

        i2cReleaseBus(&I2C_DEV);
        chThdSleepMilliseconds(100);
    }
}
