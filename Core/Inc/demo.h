/*
 * demo.h
 *
 *  Created on: Feb 12, 2024
 *      Author: gvigelet
 */

#ifndef INC_DEMO_H_
#define INC_DEMO_H_

#include "tx7332.h"

extern unsigned int reg_values[][2];

void write_demo_registers(TX7332* pT);
int verify_demo_registers(TX7332* pT);
void write_block_registers(TX7332* pT, uint8_t* data, uint16_t data_len);

#endif /* INC_DEMO_H_ */
