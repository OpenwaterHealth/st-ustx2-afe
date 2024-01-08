/*
 * i2c_slave.c
 *
 *  Created on: Jan 8, 2024
 *      Author: gvigelet
 */



#include "i2c_slave.h"

extern I2C_HandleTypeDef hi2c1;

/* Variable used to trig an address match code event */
__IO uint32_t     uwTransferRequested = 0;

void I2C_Slave_Init() {
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
	/* Transfer error in reception process */
	printf("Error Initializing I2C\r\n");
	Error_Handler();
  }
  else
  {
	  printf("I2C Slave Started\r\n");
  }
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Reset address match code event */
  uwTransferRequested = 0;

  /* Turn LED2 off: Transfer in transmission process is correct */
  HAL_GPIO_WritePin(nHB_LED_GPIO_Port, nHB_LED_Pin, GPIO_PIN_SET);
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
	uwTransferRequested = 0;

    /* Turn On LED2 */
	HAL_GPIO_WritePin(nHB_LED_GPIO_Port, nHB_LED_Pin, GPIO_PIN_SET);

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
	if(hi2c->Instance == I2C1) {
		// Your code to handle the address match event
		// Typically, you would set a flag here or initiate some data transfer
		uwTransferRequested = 1;

		// Complete the address phase of the communication
		HAL_I2C_EnableListen_IT(hi2c);
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
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave don't acknowledge it's address, Master restarts communication.
    * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    /* Turn Off LED2 */
	HAL_GPIO_WritePin(nHB_LED_GPIO_Port, nHB_LED_Pin, GPIO_PIN_RESET);

    Error_Handler();
  }
}
