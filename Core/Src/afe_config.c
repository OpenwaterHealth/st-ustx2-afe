/*
 * afe_config.c
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#include "afe_config.h"



void loadDeviceConfig(DeviceConfig_t *config)
{
	config->i2c_address = *(__IO uint32_t*)EEPROM_I2C_ADDR;
}

void writeDeviceConfig(DeviceConfig_t *config)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, EEPROM_I2C_ADDR, config->i2c_address);
	HAL_FLASHEx_DATAEEPROM_Lock();
}

void eraseDeviceConfig(void)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_I2C_ADDR);
	HAL_FLASHEx_DATAEEPROM_Lock();
}
