#include "config.h"
#include "ch.h"
#include "hal.h"
#include "eeprom.h"
#include "stm32f4xx_conf.h"
#include <string.h>
#include "controller.h"
#include "utils.h"

#define EEPROM_BASE              1000

// Global variables
uint16_t VirtAddVarTab[NB_OF_VAR];

static volatile Config config;

void config_init(void)
{
    config.CANDeviceID = 0x00;
    config.encoderType = AS5x47P;
    config.commInterface = NUNCHUK;
    config.zsmMode = BOTTOM_CLAMP;
    config.polePairs = 7;
    config.encoderZero = 173.7f;
    config.encoderInverted = true;
    config.pwmFrequency = 20000.0;
    config.currentKp = 0.1;
    config.currentKi = 50.0;
    config.maxDuty = 0.95;
    config.maxCurrent = 10.0;
    config.pllKp = 2000.0;
    config.pllKi = 20000.0;
    config.speedKp = 0.005;
    config.speedKi = 0.005;;
    config.speedKd = 0.001;
    config.motorResistance = 0.88;
    config.motorInductance = 0.000022;
    config.motorFluxLinkage = 0.00342;
    config.observerGain = 3e7;
    config.forwardCAN = false;
    config.CANStatusRate = 10;
    config.sendStatusCAN = false;

    memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

    int ind = 0;
    for (unsigned int i = 0; i < (sizeof(Config) / 2); i++) {
        VirtAddVarTab[ind++] = EEPROM_BASE + i;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    EE_Init();
}

Config* config_get_configuration(void)
{
    return &config;
}

bool config_write(void)
{
    controller_disable();
    utils_sys_lock_cnt();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

    bool is_ok = true;
    uint8_t *conf_addr = (uint8_t*)&config;
    uint16_t var;

    FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    for (unsigned int i = 0;i < (sizeof(Config) / 2);i++) {
        var = (conf_addr[2 * i] << 8) & 0xFF00;
        var |= conf_addr[2 * i + 1] & 0xFF;

        if (EE_WriteVariable(EEPROM_BASE + i, var) != FLASH_COMPLETE) {
            is_ok = false;
            break;
        }
    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
    utils_sys_unlock_cnt();

    return is_ok;
}

void config_read(Config *conf)
{
    bool is_ok = true;
    uint8_t *conf_addr = (uint8_t*)conf;
    uint16_t var;

    for (unsigned int i = 0;i < (sizeof(Config) / 2);i++) {
        if (EE_ReadVariable(EEPROM_BASE + i, &var) == 0) {
            conf_addr[2 * i] = (var >> 8) & 0xFF;
            conf_addr[2 * i + 1] = var & 0xFF;
        } else {
            is_ok = false;
            break;
        }
    }

    /*if (!is_ok) {*/
        /*conf_general_get_default_mc_configuration(conf);*/
    /*}*/
}
