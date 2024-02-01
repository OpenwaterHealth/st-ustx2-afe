/*
 * i2c_master.c
 *
 *  Created on: Jan 17, 2024
 *      Author: gvigelet
 */
#include "i2c_master.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define MAX_FOUND_ADDRESSES 5

extern I2C_HandleTypeDef hi2c2;
uint8_t selected_slave = 0xFF;
static uint8_t found_address_count = 0;
uint8_t found_addresses[MAX_FOUND_ADDRESSES];

void I2C_scan(void)
{
    // Reset the global array and counter
    memset(found_addresses, 0, MAX_FOUND_ADDRESSES );
    found_address_count = 0;

    for (uint8_t address = 0x0; address < 0x7f; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(&hi2c2, address << 1, 2, 200); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
            found_addresses[found_address_count] = address;
            found_address_count++;
        	printf("%2x ", address);
        }else{
        	printf("-- ");
        }
        if (address > 0 && (address + 1) % 16 == 0) printf("\r\n");
    }

    printf("\r\n\r\n");
    fflush(stdout);

}

uint16_t I2C_read_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr) {
    // Assuming that 'reg_addr' is an 8-bit value according to the device documentation
    uint16_t reg_value = 0x0000;
    HAL_StatusTypeDef status;
    uint8_t data_to_send = reg_addr & 0xFF; // Only one byte for the register address
    uint8_t data_to_receive[2];

    // Set the address to read from
    status = HAL_I2C_Master_Transmit(&hi2c2, i2c_addr << 1, &data_to_send, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    // Receive the data from the CDCE6214 chip
    status = HAL_I2C_Master_Receive(&hi2c2, i2c_addr << 1, data_to_receive, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    reg_value = ((uint16_t)data_to_receive[0] << 8) | data_to_receive[1];

    return reg_value;
}

uint16_t I2C_read_CDCE6214_EEPROM(uint8_t i2c_addr, uint16_t eeprom_addr) {
    // Assuming that 'reg_addr' is an 8-bit value according to the device documentation
    uint16_t reg_value = 0x0000;
    HAL_StatusTypeDef status;

    uint8_t data_to_send[2];
    data_to_send[0] = 0x00; // Register address byte
    data_to_send[1] = 0x00; // Data high byte

    uint8_t data_to_receive[2];

    status = HAL_I2C_Mem_Write(&hi2c2, i2c_addr << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, data_to_send, 2, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		// Handle error
		return 0xFFFF; // Error value
	}

    // Now read the data from NVM_RD_DATA
    status = HAL_I2C_Mem_Read(&hi2c2, i2c_addr << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, data_to_receive, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Error value
    }

    reg_value = ((uint16_t)data_to_receive[0] << 8) | data_to_receive[1];

    return reg_value;
}

bool I2C_write_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr, uint16_t reg_val) {
    // Assuming that 'reg_addr' is an 8-bit value according to the device documentation
    bool b_res = true;
    HAL_StatusTypeDef status;
    uint8_t data_to_send[3];

    // Prepare the data to send, including register address and data bytes
    data_to_send[0] = reg_addr & 0xFF; // Register address byte
    data_to_send[1] = (uint8_t)(reg_val >> 8); // Data high byte
    data_to_send[2] = (uint8_t)(reg_val & 0xFF); // Data low byte

    // Start I2C communication and send the data
    status = HAL_I2C_Master_Transmit(&hi2c2, i2c_addr << 1, data_to_send, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        b_res = false;
    }

    return b_res;
}

#if 0
uint16_t I2C_read_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr)
{
	uint16_t reg_value = 0x0000;
    HAL_StatusTypeDef status;

    uint8_t data_to_send[2];
    data_to_send[0] = (uint8_t)(reg_addr >> 8); // Register address high byte
    data_to_send[1] = (uint8_t)(reg_addr & 0xFF); // Register address low byte


    uint8_t data_to_receive[2];

    // Start I2C communication
    status = HAL_I2C_Master_Transmit(&hi2c2, i2c_addr << 1, data_to_send, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    // Receive the data from the CDCE6214 chip
    status = HAL_I2C_Master_Receive(&hi2c2, i2c_addr << 1, data_to_receive, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        return 0xFFFF; // Return an error value
    }

    reg_value = ((uint16_t)data_to_receive[0] << 8) | data_to_receive[1];

	return reg_value;
}


bool I2C_write_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr, uint16_t reg_val)
{
	bool b_res = true;
    HAL_StatusTypeDef status;
    uint8_t data_to_send[4];

    // Prepare the data to send, including slave address, register address, and data bytes
    data_to_send[0] = (uint8_t)(reg_addr >> 8); // Register address high byte
    data_to_send[1] = (uint8_t)(reg_addr & 0xFF); // Register address low byte
    data_to_send[2] = (uint8_t)(reg_val >> 8); // Data high byte
    data_to_send[3] = (uint8_t)(reg_val & 0xFF); // Data low byte

    // Start I2C communication and send the data
    status = HAL_I2C_Master_Transmit(&hi2c2, i2c_addr << 1, data_to_send, 4, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error
        b_res = false;
    }

	return b_res;
}
#endif
