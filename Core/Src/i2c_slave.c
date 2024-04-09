/*
 * i2c_slave.c
 *
 *  Created on: Mar 30, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "i2c_protocol.h"
#include "i2c_slave.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

#define I2C_DEVICE hi2c1

uint8_t rx_buffer[I2C_BUFFER_SIZE];
uint8_t tx_buffer[I2C_BUFFER_SIZE];
uint8_t status_buffer[I2C_STATUS_SIZE];

uint8_t tx_position = 0;  // 0 - status, 8 - data packet
size_t tx_bytes = 0;
static uint8_t* send_buffer = 0;
I2C_TX_Packet* data_available;

I2C_TX_Packet tx_packet;
I2C_TX_Packet rx_packet;
I2C_STATUS_Packet status_packet;

__IO uint16_t rx_count = 0;
__IO uint16_t tx_count = 0;
__IO int is_first_byte_received = 0;
__IO int countAddr = 0;
__IO int countrxCplt = 0;
__IO int countError = 0;

void I2C_Slave_Init(uint8_t addr) {


  if(addr == 0x00 || addr > 0x3F){
	  I2C_DEVICE.Init.OwnAddress1  = 0x32 << 1;  // default to 32
  }else{
	  I2C_DEVICE.Init.OwnAddress1  = addr << 1;
  }

  data_available = NULL;
  // Reinitialize the I2C peripheral with the updated configuration
  if (HAL_I2C_Init(&I2C_DEVICE) != HAL_OK) {
	  // Handle the error if reinitialization fails
	  Error_Handler();
  }

  // clear header
  memset(tx_buffer, 0, I2C_BUFFER_SIZE);


  status_packet.id = 00;
  status_packet.status = 0x00; // processing
  status_packet.cmd = 0x00;
  status_packet.data_len = 0;
  i2c_status_packet_toBuffer(&status_packet, status_buffer);

  if(HAL_I2C_EnableListen_IT(&I2C_DEVICE) != HAL_OK) {
	  // Handle the error if reinitialization fails
	  Error_Handler();
  }

}

void i2c_print_info() {
    //uint32_t timing = I2C_DEVICE.Init.Timing;
    //uint32_t pclk = HAL_RCC_GetPCLK1Freq(); // Get the peripheral clock frequency

    // Calculate the I2C speed in Hz
    //uint32_t i2c_speed = pclk / ((timing & 0xFFFF) + 1);

    printf("I2C Speed: %d kHz\r\n", 400); // Print the I2C speed in kHz
    printf("I2C Slave Addr: 0x%02x\r\n\r\n", (uint8_t)(I2C_DEVICE.Init.OwnAddress1 >> 1));
}


void I2C_Process() {

}

bool set_status_buffer(I2C_STATUS_Packet* status)
{
	bool ret = false;

	if(i2c_status_packet_toBuffer(status,status_buffer)>0)
	{
		ret = true;
	}

	return ret;
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{

	if(TransferDirection == I2C_DIRECTION_TRANSMIT)
	{
		if(is_first_byte_received == 0)
		{
			rx_count = 0;
			countAddr++;
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rx_buffer + rx_count, 1, I2C_FIRST_FRAME);
		}
	}
	else
	{
		tx_count = 0;
		tx_position = rx_buffer[0];
		tx_bytes = 0;
		if(tx_position == 0)
		{
			// read status
			status_packet.data_len = rx_packet.pkt_len;
			tx_bytes = i2c_status_packet_toBuffer(&status_packet, status_buffer);  // update status packet
			printf("Read Status %d\r\n", tx_bytes);
			send_buffer = status_buffer;
		}else{
			// read buffer
			tx_bytes = i2c_packet_toBuffer(&rx_packet, tx_buffer);
			printf("Read Data %d\r\n", tx_bytes);
			send_buffer = tx_buffer;
		}

		HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, send_buffer, tx_bytes, I2C_FIRST_AND_LAST_FRAME);
	}
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	if(I2cHandle->Instance == I2C_DEVICE.Instance) {
		if(is_first_byte_received == 1)
		{
			is_first_byte_received = 0;
		}
		else
		{
			printf("send NAK\r\n");
			__HAL_I2C_GENERATE_NACK(I2cHandle);
		}
#if 0
		// can use this incase we don't know how much the master wants to read and generate a nak at the end of the buffer.
		tx_count++;
		HAL_I2C_Slave_Seq_Transmit_IT(I2cHandle, send_buffer+tx_count, 1, I2C_NEXT_FRAME);
#endif
	}
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Reset address match code event */
	if(I2cHandle->Instance == I2C_DEVICE.Instance) {
		if(is_first_byte_received == 0)
		{
			rx_count++;
			is_first_byte_received = 1;
			uint8_t bytes_left = rx_buffer[0]-1;

			HAL_I2C_Slave_Seq_Receive_IT(I2cHandle, rx_buffer + rx_count, bytes_left, I2C_LAST_FRAME);
		}
		else
		{
			rx_count = rx_buffer[0];
			is_first_byte_received=0;
			// process data
			i2c_packet_fromBuffer(rx_buffer, &rx_packet);
			status_packet.id = rx_packet.id;
			status_packet.cmd = rx_packet.cmd;
			status_packet.status = 0xA5;
			// printBuffer(rx_buffer, rx_count);
			// process or send for processing
			data_available = &rx_packet;
		}
	}
	else
	{
		printf("UNHANDLED I2C Instance\r\n");
	}
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  countError++;
  uint32_t errorcode = HAL_I2C_GetError(I2cHandle);
  if (errorcode == 4)  // AF error
  {
	__HAL_I2C_CLEAR_FLAG(I2cHandle, I2C_FLAG_AF); //clear AF flag
	if(tx_count == 0) //error is while slave is receiving
	{
		//process_data();
		rx_count = 0;
	}
	else // error while slave is transmitting
	{
		tx_count = 0;
	}
  }
  else if (errorcode == 1)  // BERR Error
  {
	  printf("HAL_I2C_ErrorCallback ERR: 0x%08lX Resetting devide\r\n", errorcode);
	  HAL_I2C_DeInit(I2cHandle);
	  HAL_I2C_Init(I2cHandle);

	  //enable_receive_header();
  }else{
	  printf("HAL_I2C_ErrorCallback ERR: 0x%08lX\r\n", errorcode);
	  Error_Handler();
  }
  HAL_I2C_EnableListen_IT(I2cHandle);
}
