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

enum DeviceState {
  // initial state
  STATE_INITIAL = 0,
  // receiving packet
  STATE_RECEIVING_PACKET,
  // sending packet
  STATE_SENDING_PACKET,
  // packet complete
  STATE_COMPLETED_PACKET
};

// Function prototypes for I2C slave operations
void I2C_Slave_Init(uint8_t addr);
void I2C_Slave_ProcessData(uint8_t data);


#endif /* INC_I2C_SLAVE_H_ */
