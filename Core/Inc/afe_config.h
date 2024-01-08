/*
 * afe_config.h
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#ifndef INC_AFE_CONFIG_H_
#define INC_AFE_CONFIG_H_

#include "main.h"
#include <stdint.h>

#define EEPROM_USER_START_ADDR   DATA_EEPROM_BANK2_BASE  /* Start @ of EEPROM area */
#define EEPROM_USER_END_ADDR     DATA_EEPROM_BANK2_END   /* End @ of EEPROM area */
#define EEPROM_I2C_ADDR 		 DATA_EEPROM_BANK2_BASE
// Configuration structure
typedef struct {
    uint8_t i2c_address;
    // Add more configuration parameters if needed
} DeviceConfig_t;


// Functions for configuration
void loadDeviceConfig(DeviceConfig_t *config);
void writeDeviceConfig(DeviceConfig_t *config);
void eraseDeviceConfig(void);

#endif /* INC_AFE_CONFIG_H_ */
