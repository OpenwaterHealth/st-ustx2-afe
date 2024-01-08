/*
 * i2c_slave.h
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

// Function prototypes for I2C slave operations
void I2C_Slave_Init(void);
void I2C_Slave_ProcessData(uint8_t data);


#endif /* INC_I2C_SLAVE_H_ */
