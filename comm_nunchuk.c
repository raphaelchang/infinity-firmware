#include "comm_nunchuk.h"

#include "ch.h"
#include "hal.h"
#include "hw_conf.h"
#include "comm_usb.h"
#include <string.h>

#include "stm32f4xx_conf.h"

static const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

static uint8_t rxbuf[10];
static uint8_t txbuf[10];
static msg_t status = MSG_OK;
static i2caddr_t addr = 0x52;
static systime_t tmo = MS2ST(10);

void init_I2C1(void){

    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    // enable APB1 peripheral clock for I2C1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // configure I2C1 
    I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
    I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

    // enable I2C1
    I2C_Cmd(I2C1, ENABLE);
}

/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
    // wait until I2C1 is not busy anymore
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    // Send I2C1 START condition 
    I2C_GenerateSTART(I2Cx, ENABLE);

    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    // Send slave Address for write 
    I2C_Send7bitAddress(I2Cx, address, direction);

    /* wait for I2C1 EV6, check if 
     * either Slave has acknowledged Master transmitter or
     * Master receiver mode, depending on the transmission
     * direction
     */ 
    if(direction == I2C_Direction_Transmitter){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if(direction == I2C_Direction_Receiver){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2C_SendData(I2Cx, data);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
    // enable acknowledge of recieved data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
    // disabe acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    uint8_t data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
    // Send I2C1 STOP Condition 
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

void comm_nunchuk_init(void)
{
    i2cAcquireBus(&I2C_DEV); 

    palSetPadMode(I2C_GPIO, I2C_SCL_PIN,
            PAL_MODE_ALTERNATE(4) |
            PAL_STM32_OTYPE_OPENDRAIN |
            PAL_STM32_PUPDR_PULLUP |
            PAL_STM32_OSPEED_MID1);
    palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
            PAL_MODE_ALTERNATE(4) |
            PAL_STM32_OTYPE_OPENDRAIN |
            PAL_STM32_PUPDR_PULLUP |
            PAL_STM32_OSPEED_MID1);

    init_I2C1();
    i2cStart(&I2C_DEV, &i2ccfg);
    i2cReleaseBus(&I2C_DEV);
}

void comm_nunchuk_update(void)
{
    bool is_ok = true;

    txbuf[0] = 0xF0;
    txbuf[1] = 0x55;
    /*I2C_start(I2C1, addr<<1, I2C_Direction_Transmitter);*/
    /*I2C_write(I2C1, txbuf[0]);*/
    /*I2C_write(I2C1, txbuf[1]);*/
    /*I2C_stop(I2C1);*/
    i2cAcquireBus(&I2C_DEV);
    status = i2cMasterTransmitTimeout(&I2C_DEV, addr, txbuf, 2, rxbuf, 0, tmo);
    i2cReleaseBus(&I2C_DEV);
    is_ok = status == MSG_OK;

    /*USB_PRINT("Update nunchuk 0 %d\n", status);*/
    if (is_ok) {
        txbuf[0] = 0xFB;
        txbuf[1] = 0x00;
        i2cAcquireBus(&I2C_DEV);
        status = i2cMasterTransmitTimeout(&I2C_DEV, addr, txbuf, 2, rxbuf, 0, tmo);
        i2cReleaseBus(&I2C_DEV);
        is_ok = status == MSG_OK;
    }
    /*USB_PRINT("Update nunchuk 1 %d\n", i2cGetErrors(&I2C_DEV));*/
    return;

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
        USB_PRINT("Resetting I2C %d\n", 1);
        i2cAcquireBus(&I2C_DEV);
        i2cStop(&I2C_DEV);

        palSetPadMode(I2C_GPIO, I2C_SCL_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_PUPDR_PULLUP |
                PAL_STM32_OSPEED_MID1);

        palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_PUPDR_PULLUP |
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
                PAL_STM32_PUPDR_PULLUP |
                PAL_STM32_OSPEED_MID1);

        palSetPadMode(I2C_GPIO, I2C_SDA_PIN,
                PAL_MODE_ALTERNATE(4) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_PUPDR_PULLUP |
                PAL_STM32_OSPEED_MID1);

        I2C_DEV.state = I2C_STOP;
        i2cStart(&I2C_DEV, &i2ccfg);

        i2cReleaseBus(&I2C_DEV);
        chThdSleepMilliseconds(100);
    }
}
