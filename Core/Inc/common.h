/*
 * common.h
 *
 *  Created on: Apr 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_


typedef enum {
	CMD_TURN_OFF_LED = 0x02,
	CMD_TURN_ON_LED = 0x03,
	CMD_HB_LED = 0x04,
	CMD_TX_DEMO = 0x05,
	CMD_TX_TEST = 0x06,
	AFE_CMD_TOGGLE_LED = 0xCF
} I2C_USTX_AFE_Command;

#endif /* INC_COMMON_H_ */
