/*
 * demo.c
 *
 *  Created on: Feb 12, 2024
 *      Author: gvigelet
 */
#include "demo.h"
#include "main.h"
#include <stdio.h>

unsigned int reg_values[][2] = {
        {0x00, 0x00000000},
        {0x01, 0x00000000},
        {0x06, 0x00000000},
        {0x0B, 0x00000000},
        {0x0C, 0x00000000},
        {0x0F, 0x00000000},
        {0x14, 0x00000000},
        {0x15, 0x00000000},
        {0x16, 0x00000000},
        {0x17, 0x00000000},
        {0x18, 0x02000003},
        {0x19, 0x00000744},
        {0x1A, 0x00000000},
        {0x1B, 0x00000000},
        {0x1E, 0x00000000},
        {0x1F, 0x00000000},
        {0x20, 0x1FFF1770},
        {0x21, 0x1FFF1770},
        {0x22, 0x0E1004B0},
        {0x23, 0x0E1004B0},
        {0x24, 0x1C2012C0},
        {0x25, 0x1C2012C0},
        {0x26, 0x09600000},
        {0x27, 0x09600000},
        {0x28, 0x1FFF1770},
        {0x29, 0x1FFF1770},
        {0x2A, 0x0E1004B0},
        {0x2B, 0x0E1004B0},
        {0x2C, 0x1C2012C0},
        {0x2D, 0x1C2012C0},
        {0x2E, 0x09600000},
        {0x2F, 0x09600000},
        {0x120, 0xF1F2F2F2},
        {0x121, 0x0007F1F1},
    };

void write_demo_registers(TX7332* pT)
{
	int num_registers = sizeof(reg_values) / sizeof(reg_values[0]);
	for (int i = 0; i < num_registers; i++) {
        unsigned int reg_address = reg_values[i][0];
        unsigned int expected_value = reg_values[i][1];
		TX7332_WriteReg(pT, reg_address, expected_value);
	}
}

int verify_demo_registers(TX7332* pT)
{
	int bSuccess = 1;
	int num_registers = sizeof(reg_values) / sizeof(reg_values[0]);
	for (int i = 0; i < num_registers; i++) {
        unsigned int reg_address = reg_values[i][0];
        unsigned int expected_value = reg_values[i][1];
        unsigned int actual_value = TX7332_ReadReg(pT, reg_address);

        if (actual_value != expected_value) {
            printf("==> Mismatch at index 0x%04X: 0x%08X, 0x%08X\r\n", (uint16_t)reg_address, actual_value, expected_value);
        	bSuccess = 0;
        }else{
            printf("Match at index 0x%04X: 0x%08X, 0x%08X\r\n", (uint16_t)reg_address, actual_value, expected_value);
        }
	}
	return bSuccess;
}


void write_block_registers(TX7332* pT, uint8_t* data, uint16_t data_len)
{
	uint8_t eFlag = 0;
	int num_values = data_len / 8;
    // Check if the data contains a valid number of values
    if (data_len % 8 != 0) {
        return;
    }
    // Loop through the data and convert it into pairs of unsigned int values
	unsigned int temp_values[num_values][2];

	for (int i = 0; i < num_values; i++) {
		temp_values[i][0] = (data[i * 8] << 24) | (data[i * 8 + 1] << 16) | (data[i * 8 + 2] << 8) | data[i * 8 + 3];
		temp_values[i][1] = (data[i * 8 + 4] << 24) | (data[i * 8 + 5] << 16) | (data[i * 8 + 6] << 8) | data[i * 8 + 7];
	}

    // Now you have temp_values as an array of pairs of unsigned int values
    // You can compare it against reg_values
    for (int i = 0; i < num_values; i++) {
        if (i < sizeof(reg_values) / sizeof(reg_values[0])) {
            if (temp_values[i][0] == reg_values[i][0] && temp_values[i][1] == reg_values[i][1]) {
                unsigned int reg_address = (unsigned int)temp_values[i][0];
                unsigned int expected_value = (unsigned int)temp_values[i][1];
        		TX7332_WriteReg(pT, reg_address, expected_value);
            } else {
                printf("Mismatch at index %d: %08X, %08X\r\n", i, temp_values[i][0], temp_values[i][1]);
            	eFlag = 1;
            }
        } else {
            printf("Data received contains more values than reg_values.\r\n");
        	eFlag = 1;
            break;
        }
    }

    if(eFlag == 1){
        printf("Error flag set.\r\n");
    	HAL_GPIO_WritePin(nHB_LED_GPIO_Port, nHB_LED_Pin, GPIO_PIN_SET);
    }
}


