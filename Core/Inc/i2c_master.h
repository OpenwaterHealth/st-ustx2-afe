/*
 * i2c_master.h
 *
 *  Created on: Jan 17, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

#define CDCE6214_I2C_ADDR_1 0x68
#define CDCE6214_I2C_ADDR_0 0x69

void I2C_scan(void);
uint16_t I2C_read_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr);
bool I2C_write_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr, uint16_t reg_val);
uint16_t I2C_read_CDCE6214_EEPROM(uint8_t i2c_addr, uint16_t eeprom_addr);

#endif /* INC_I2C_MASTER_H_ */
