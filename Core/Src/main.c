/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "i2c_slave.h"
#include "i2c_master.h"
#include "afe_config.h"
#include "config_CDCE6214_64MHZ.h"
#include "utils.h"
#include "logging.h"
#include "tx7332.h"
#include "demo.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_usart5_rx;
DMA_HandleTypeDef hdma_usart5_tx;

/* USER CODE BEGIN PV */
DeviceConfig_t myConfig;

TX7332 tx[2];

static uint8_t FIRMWARE_VERSION_DATA[3] = {1, 0, 0};
static uint32_t id_words[3] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PrintI2CSpeed(I2C_HandleTypeDef *hi2c)
{
  uint32_t timing = hi2c->Init.Timing;
  uint32_t pclk = HAL_RCC_GetPCLK1Freq(); // Get the peripheral clock frequency

  // Calculate the I2C speed in Hz
  uint32_t i2c_speed = pclk / ((timing & 0xFFFF) + 1);

  printf("I2C Speed: %ld kHz\r\n", i2c_speed / 10); // Print the I2C speed in kHz
  printf("I2C Slave Addr: 0x%02x\r\n\r\n", (uint8_t)(hi2c->Init.OwnAddress1 >> 1));
}

static bool ConfigureClock()
{

	  HAL_Delay(25);
	  I2C_write_CDCE6214_reg(0x67, 0x0000, 0x1000);
	  HAL_Delay(25);
	  I2C_write_CDCE6214_reg(0x67, 0x000F, 0x5020);
	  HAL_Delay(25);

	  printf("Configuring Clock chip\r\n");
	  // Calculate the number of elements in the array
	  size_t num_elements = sizeof(blur6214_64mhz_values) / sizeof(uint32_t);

	  // Iterate through the array and split each uint32_t value into two uint16_t values
	  for (size_t i = 0; i < num_elements; i++)
	  {
	    uint32_t value = blur6214_64mhz_values[i];

	    // Split the value into upper and lower words
	    uint16_t reg_addr = (uint16_t)(value >> 16); // Upper word is reg_addr
	    uint16_t reg_value = (uint16_t)value;        // Lower word is reg_value

	    // Print the split values
	    if (!I2C_write_CDCE6214_reg(0x67, reg_addr, reg_value))
	    {
	      printf("failed Index %zu: reg_addr = 0x%04X, reg_value = 0x%04X\r\n", i, reg_addr, reg_value);
	      return false;
	    }
	    HAL_Delay(1);
	  }
	  HAL_Delay(100);
	  I2C_write_CDCE6214_reg(0x67, 0x0000, 0x1110);
	  HAL_Delay(100);
	  I2C_write_CDCE6214_reg(0x67, 0x0000, 0x1100);
	  HAL_Delay(50);

	  return true;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  I2C_TX_Packet ret_data;
  uint8_t ret_data_buffer[I2C_BUFFER_SIZE] = {0};
  uint16_t address;
  uint32_t value;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  loadDeviceConfig(&myConfig);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART5_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  printf("\033c");

  init_dma_logging();
  printf("Openwater USTX2 AFE Development v1.0.3\r\n\r\n");
  printf("EEPROM I2C: 0x%02x\r\n", myConfig.i2c_address);
  printf("CPU Clock Frequency: %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);

  // Initializing I2C Slave
  data_available = NULL;
  I2C_Slave_Init(myConfig.i2c_address);
  PrintI2CSpeed(&hi2c1);

  printf("Scanning Local I2C bus\r\n");
  I2C_scan();
  ConfigureClock();

  printf("Initializing TX7332\r\n");
  HAL_GPIO_WritePin(GPIOC, RESET_L_Pin | CW_EN_Pin | STDBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, DSEL0_Pin | DSEL1_Pin | TR_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, CS_TXA_Pin | CS_TXB_Pin, GPIO_PIN_RESET); // TODO: Verify initial state

  // reset TX7332
  TX7332_Reset();
  HAL_Delay(25);

  // configure CS for TX7332
  TX7332_Init(&tx[0], CS_TXA_GPIO_Port, CS_TXA_Pin);
  TX7332_Init(&tx[1], CS_TXB_GPIO_Port, CS_TXB_Pin);

  HAL_Delay(10);

  HAL_GPIO_WritePin(CW_EN_GPIO_Port, CW_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TR_EN_GPIO_Port, TR_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DSEL0_GPIO_Port, DSEL0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DSEL1_GPIO_Port, DSEL1_Pin, GPIO_PIN_RESET);

  //  printf("Writing Demo Registers\r\n");
  //  write_demo_registers(&tx[0]);
  //  HAL_Delay(10);

#ifdef RUN_TESTS

  for (uint16_t x = 0; x < 86; x++)
  {
    printf("Read R%d: ", x);
    uint16_t reg_val = I2C_read_CDCE6214_reg(0x67, x);
    printf("0x%04x\r\n", reg_val);
  }

  HAL_Delay(25);

  if (crc_test() == 1)
  {
    printf("CRC Test Failed\r\n\r\n");
  }
  else
  {
    printf("CRC Test Passed\r\n\r\n");
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    memset(ret_data_buffer, 0, I2C_BUFFER_SIZE);
    memset((uint8_t *)&ret_data, 0, sizeof(ret_data));
    if (data_available)
    {
      // Process command
      status_packet->id = data_available->id;
      status_packet->cmd = data_available->cmd;
      status_packet->status = 0xFF;
      status_packet->data_len = 0;
      
      // print received packet
      // i2c_tx_packet_print(data_available);
      switch (data_available->cmd)
      {
      case OW_CMD_PING:
        // printf("AFE Ping\r\n");
        status_packet->cmd = OW_CMD_PONG;
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      case OW_CMD_PONG:
    	  // printf("AFE Pong\r\n");
        status_packet->cmd = OW_CMD_PING;
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      case OW_CMD_TOGGLE_LED:
    	  // printf("Toggling LED\r\n");
        HAL_GPIO_TogglePin(nHB_LED_GPIO_Port, nHB_LED_Pin);
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      case OW_CMD_ECHO:
    	  // printf("AFE Echo\r\n");
        status_packet->status = 0x00;
        ret_data.cmd = data_available->cmd;
        ret_data.id = data_available->id;
        ret_data.reserved = 0;
        if (data_available->data_len > 0)
        {
          ret_data.data_len = data_available->data_len;
          ret_data.pData = data_available->pData;
        }
        else
        {
          ret_data.data_len = 0;
          ret_data.pData = NULL;
          status_packet->data_len = 0;
        }
        set_transmit_buffer(&ret_data, data_available->id, data_available->cmd, OW_CODE_SUCCESS);

        break;
      case OW_CMD_VERSION:
    	  // printf("AFE Version\r\n");
        status_packet->status = 0x00;
        ret_data.cmd = data_available->cmd;
        ret_data.id = data_available->id;
        ret_data.reserved = 0;
        ret_data.data_len = sizeof(FIRMWARE_VERSION_DATA);
        ret_data.pData = FIRMWARE_VERSION_DATA;
        set_transmit_buffer(&ret_data, data_available->id, data_available->cmd, OW_CODE_SUCCESS);
        break;
      case OW_CMD_HWID:
    	  // printf("AFE CHIP ID\r\n");
        status_packet->status = 0x00;
        ret_data.cmd = data_available->cmd;
        ret_data.id = data_available->id;
        id_words[0] = HAL_GetUIDw0();
        id_words[1] = HAL_GetUIDw1();
        id_words[2] = HAL_GetUIDw2();

        // Print the contents of id_words array in hexadecimal format
        // printf("id_words[0]: 0x%lx\r\n", id_words[0]);
        // printf("id_words[1]: 0x%lx\r\n", id_words[1]);
        // printf("id_words[2]: 0x%lx\r\n", id_words[2]);

        ret_data.data_len = sizeof(id_words);
        ret_data.pData = (uint8_t *)id_words;
        set_transmit_buffer(&ret_data, data_available->id, data_available->cmd, OW_CODE_SUCCESS);
        break;
      case OW_CMD_RESET:
    	  // printf("AFE RESET\r\n");
        status_packet->cmd = OW_CMD_RESET;
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        HAL_Delay(1);
        // Reset the board
        NVIC_SystemReset();
        break;
      case OW_AFE_ENUM_TX7332:
    	  // printf("Enumerate TX7332 ICs %d \r\n", ARRAY_SIZE(tx));
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        status_packet->reserved = (uint8_t)ARRAY_SIZE(tx);
        set_transmit_buffer(NULL, data_available->id, data_available->cmd, OW_CODE_SUCCESS);
        break;
      case OW_TX7332_DEMO:
    	// printf("Writing Demo TX7332 [0] Register Set\r\n");
        write_demo_registers(&tx[0]);
        // printf("Writing Demo TX7332 [1] Register Set\r\n");
        write_demo_registers(&tx[1]);
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        // HAL_Delay(10);
        // printf("Verifying Demo TX7332 Register Set\r\n");
        // verify_demo_registers(&tx[0]);
        break;
      case OW_TX7332_WREG:  //
        // printf("Write REG TX[%d] \r\n", data_available->reserved); //0x%04x : 0x08x
        if(data_available->reserved > 1){
            status_packet->status = OW_CODE_IDENT_ERROR;
            status_packet->data_len = 0;
        	break;
        }

        if(data_available->data_len == 6)
        {
			// Unpack 16-bit address (first 2 bytes, little-endian)
			address = data_available->pData[0] | (data_available->pData[1] << 8);
			// Unpack 32-bit value (next 4 bytes, little-endian)
			value = data_available->pData[2] | (data_available->pData[3] << 8) | (data_available->pData[4] << 16) | (data_available->pData[5] << 24);
			// printf("Address: 0x%04x, Value: 0x%08lx\r\n", address, value);
			TX7332_WriteReg(&tx[data_available->reserved], address, value);
        }else{
        	printf("Invalid data \r\n");
            status_packet->status = OW_CODE_DATA_ERROR;
            status_packet->data_len = 0;
        	break;
        }

        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      case OW_TX7332_RREG:
        // printf("Read REG TX[%d] \r\n", data_available->reserved); //0x%04x : 0x08x
        if(data_available->reserved > 1){
            status_packet->status = OW_CODE_IDENT_ERROR;
            status_packet->data_len = 0;
        	break;
        }

        if(data_available->data_len == 2)
        {
			// Unpack 16-bit address (first 2 bytes, little-endian)
			address = data_available->pData[0] | (data_available->pData[1] << 8);
			// Set value to 0 before read
			value = 0;
			// printf("Address: 0x%04x\r\n", address);
			value = TX7332_ReadReg(&tx[data_available->reserved], address);

            // Package response
	        ret_data_buffer[0] = value & 0xFF;
	        ret_data_buffer[1] = (value >> 8) & 0xFF;
	        ret_data_buffer[2] = (value >> 16) & 0xFF;
	        ret_data_buffer[3] = (value >> 24) & 0xFF;

	        ret_data.cmd = data_available->cmd;
	        ret_data.id = data_available->id;
	        ret_data.reserved = data_available->reserved;
	        ret_data.data_len = sizeof(value);
	        ret_data.pData = ret_data_buffer;
	        set_transmit_buffer(&ret_data, data_available->id, data_available->cmd, OW_CODE_SUCCESS);

        }else{
        	printf("Invalid data \r\n");
            status_packet->status = OW_CODE_DATA_ERROR;
            status_packet->data_len = 0;
        	break;
        }

        break;
      case OW_TX7332_WBLOCK:
        printf("Write Block\r\n");
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      case OW_TX7332_RBLOCK:
        printf("Read Block\r\n");
        status_packet->status = 0x00;
        status_packet->data_len = 0;
        break;
      default:
        printf("Unknown Command: 0x%02x\r\n", data_available->cmd);
        break;
      }
      data_available = NULL;
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 4129;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0xFFFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 100;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM21 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
}

/**
 * @brief USART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DSEL0_Pin | DSEL1_Pin | TR_EN_Pin | CW_EN_Pin | STDBY_Pin | RESET_L_Pin | CS_TXA_Pin | CS_TXB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nHB_LED_Pin | READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : nINTERRUPT_Pin PA3 nESTOP_Pin */
  GPIO_InitStruct.Pin = nINTERRUPT_Pin | GPIO_PIN_3 | nESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DSEL0_Pin DSEL1_Pin TR_EN_Pin CW_EN_Pin
                           STDBY_Pin RESET_L_Pin CS_TXA_Pin CS_TXB_Pin */
  GPIO_InitStruct.Pin = DSEL0_Pin | DSEL1_Pin | TR_EN_Pin | CW_EN_Pin | STDBY_Pin | RESET_L_Pin | CS_TXA_Pin | CS_TXB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_GOOD_Pin */
  GPIO_InitStruct.Pin = PWR_GOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_GOOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB15
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : nHB_LED_Pin READY_Pin */
  GPIO_InitStruct.Pin = nHB_LED_Pin | READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_TIM22;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
