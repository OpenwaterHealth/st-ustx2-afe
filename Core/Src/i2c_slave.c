/*
 * i2c_slave.c
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */

#include "i2c_slave.h"
#include <string.h>

#define RXBUFFERSIZE 8

extern I2C_HandleTypeDef hi2c1;

#define I2C_DEVICE hi2c1

/* Variable used to trig an address match code event */
__IO uint32_t uwTransferRequested;

uint8_t aRxBuffer[RXBUFFERSIZE];

static enum DeviceState state = STATE_INITIAL;

#ifdef DEBUG_COMMS
static void print_rx_buffer()
{
	printf("\r\nBuffer\r\n\r\n");
	for (uint32_t i = 0; i < RXBUFFERSIZE; i++) {
		printf("%02X ", aRxBuffer[i]); // Print each byte in hexadecimal format
	}
	printf("\r\n\r\n"); // Print a newline character to separate the output

}
#endif

void I2C_Slave_Init(uint8_t addr) {

  memset(aRxBuffer, 0, RXBUFFERSIZE);
  uwTransferRequested = 1;

  if(addr == 0x00 || addr > 0x3F){
	  I2C_DEVICE.Init.OwnAddress1  = 0x32 << 1;  // default to 32
  }else{
	  I2C_DEVICE.Init.OwnAddress1  = addr << 1;
  }

  // Reinitialize the I2C peripheral with the updated configuration
  if (HAL_I2C_Init(&I2C_DEVICE) != HAL_OK) {
	  // Handle the error if reinitialization fails
	  Error_Handler();
  }

  if(HAL_I2C_Slave_Receive_IT(&I2C_DEVICE, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
	/* Transfer error in reception process */
	printf("Error Initializing I2C\r\n");
	Error_Handler();
  }

  printf("I2C Slave Started\r\n");
}

void I2C_Slave_Run()
{
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
#ifdef DEBUG_COMMS
	printf("HAL_I2C_AddrCallback\r\n");
#endif
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
#ifdef DEBUG_COMMS
	printf("HAL_I2C_SlaveTxCpltCallback\r\n");
#endif
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{;
#ifdef DEBUG_COMMS
	printf("HAL_I2C_SlaveRxCpltCallback STATE: 0x%02X\r\n", state);
#endif
	/* Reset address match code event */
	if(I2cHandle->Instance == I2C_DEVICE.Instance) {
		/* Turn On LED2 */
		if(state == STATE_INITIAL){
			if(aRxBuffer[0] == 0xA5 && aRxBuffer[RXBUFFERSIZE-1] == 0x5A)
			{
				CommandQueue_Enqueue(&commandQueue, aRxBuffer[1]);
#ifdef DEBUG_COMMS
				printf("Packet Recieved\r\n");
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				print_rx_buffer();
#endif
			}

			memset(aRxBuffer, 0, RXBUFFERSIZE);
			if(HAL_I2C_Slave_Receive_IT(&I2C_DEVICE, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
			{
				/* Transfer error in reception process */
				printf("Error Initializing I2C\r\n");
				Error_Handler();
			}
		}
	}
	else
	{
		printf("UNHANDLED I2C Instance\r\n");
	}
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
#ifdef DEBUG_COMMS
	printf("HAL_I2C_ListenCpltCallback STATE: 0x%02X\r\n", state);
#endif
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
  uint32_t errorcode = HAL_I2C_GetError(I2cHandle);
  if (errorcode == 4)  // AF error
  {

	  memset(aRxBuffer, 0, RXBUFFERSIZE);
	  if(HAL_I2C_Slave_Receive_IT(&I2C_DEVICE, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
		/* Transfer error in reception process */
		printf("Error Initializing I2C\r\n");
		Error_Handler();
	  }
  }
  else if (errorcode == 1)  // BERR Error
  {
	  printf("HAL_I2C_ErrorCallback ERR: 0x%08lX Resetting devide\r\n", errorcode);
	  HAL_I2C_DeInit(I2cHandle);
	  HAL_I2C_Init(I2cHandle);

	  memset(aRxBuffer, 0, RXBUFFERSIZE);
	  if(HAL_I2C_Slave_Receive_IT(&I2C_DEVICE, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
		/* Transfer error in reception process */
		printf("Error Initializing I2C\r\n");
		Error_Handler();
	  }
  }else{
	  printf("HAL_I2C_ErrorCallback ERR: 0x%08lX\r\n", errorcode);
	  Error_Handler();
  }
}
