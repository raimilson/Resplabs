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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	INA219_REG_BUSVOLTAGE					(0x02)
#define	INA219_REG_SHUNTVOLTAGE					(0x01)
#define	INA219_CONFIG				        	(0x00)

#define INA220_ADDR 		0x008A

#define DATA_EEPROM_BASE_ADDR ((uint32_t)0x08080000) /* Data EEPROM base address */

#define DATA_EEPROM_END_ADDR ((uint32_t)0x080803FF) /* Data EEPROM end address */

#define CONFIG ((Eeprom_registers *) DATA_EEPROM_BASE_ADDR)

#define Charging_TABLE_SIZE 51

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

osThreadId defaultTaskHandle;
osThreadId LEDHandle;
osThreadId ButtonHandle;
osThreadId INA220Handle;
/* USER CODE BEGIN PV */
uint8_t ButtonState = 0;

uint8_t bus_reg[2] = {0x01,0x01};
uint8_t bus_reg_2[2] = {0x02,0x02};
uint8_t bus_voltage_data[2];
uint16_t bus_voltage = 0;
float bus_voltag_f = 0;

uint8_t shunt_voltage_data[2];
uint16_t shunt_voltage = 0;

uint32_t Eeprom_data1 = 0;
uint32_t data_to_write = 0;

uint32_t addr = 0;

uint16_t CurrentDraw = 0;

uint8_t status_charger = 0;

uint8_t soc = 0; // Percentage of battery left

uint8_t Saved = 0;

uint8_t updatedValues = 0;

uint8_t setInitialCapacity = 0;

uint8_t ChargerState = 0;

float ampereHour = 0;
float batteryCap = 12500;  // Total Capacity of the Battery in mA

uint16_t Real_capacity=0;

const int _Charger_lookup_table[Charging_TABLE_SIZE][2] = {
    {10269, 80},
    {10699, 330},
    {10883, 580},
    {10957, 830},
    {10994, 1080},
    {11028, 1330},
    {11072, 1580},
    {11127, 1830},
    {11188, 2080},
    {11249, 2330},
    {11307, 2580},
    {11360, 2830},
    {11406, 3080},
    {11448, 3330},
    {11488, 3580},
    {11527, 3830},
    {11567, 4080},
    {11610, 4330},
    {11657, 4580},
    {11707, 4830},
    {11760, 5080},
    {11816, 5330},
    {11872, 5580},
    {11928, 5830},
    {11983, 6080},
    {12036, 6330},
    {12087, 6580},
    {12136, 6830},
    {12183, 7080},
    {12229, 7330},
    {12275, 7580},
    {12319, 7830},
    {12363, 8080},
    {12406, 8330},
    {12447, 8580},
    {12486, 8830},
    {12521, 9080},
    {12551, 9330},
    {12575, 9580},
    {12594, 9830},
    {12606, 10080},
    {12613, 10330},
    {12616, 10580},
    {12617, 10830},
    {12617, 11080},
    {12619, 11330},
    {12623, 11580},
    {12628, 11830},
    {12634, 12080},
    {12637, 12330},
    {12637, 12500},
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM21_Init(void);
void StartDefaultTask(void const * argument);
void StartLED(void const * argument);
void StartButton(void const * argument);
void StartINA220(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t lookup(uint16_t input_value) {
    uint8_t i;
    uint16_t min_diff = UINT16_MAX;
    uint16_t output_value = 0;

    // Search for the closest matching value in the table
    for (i = 0; i < Charging_TABLE_SIZE; i++) {
        uint16_t diff = abs(input_value - _Charger_lookup_table[i][0]);
        if (diff < min_diff) {
            min_diff = diff;
            output_value = _Charger_lookup_table[i][1];
        }
    }

    return output_value ;

}

void clear_indicator_leds()
{
	HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);



}
void DisplayLevel()

{
		if(soc <= 10)	// 1000mV
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);
			for(int i = 0; i < 10; i++)

			{
				HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
				HAL_Delay(200);

			}
		}

		else if(soc > 10 && soc <= 20 )
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);

		}

		else if(soc > 20 && soc <= 40 )
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);

		}

		else if(soc > 40 && soc <= 60 )
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);

		}

		else if (soc > 60 && soc <= 80)
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_RESET);


		}

		else if (soc > 80)
		{
			HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_05_GPIO_Port, LED_05_Pin, GPIO_PIN_SET);


		}

	osDelay(3000);
	clear_indicator_leds();


}
void set_indicator_leds_blink()


{

	if(soc <= 20)	// 1000mV
	{
		HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
		HAL_Delay(200);

	}

	else if(soc > 20 && soc <= 40 )
	{
		HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_02_GPIO_Port, LED_02_Pin);
		HAL_Delay(200);


	}

	else if(soc > 40 && soc <= 60 )
	{

		HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_03_GPIO_Port, LED_03_Pin);
		HAL_Delay(200);



	}

	else if (soc > 60 && soc <= 80)
	{
		HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_04_GPIO_Port, LED_04_Pin);
		HAL_Delay(200);


	}

	else if (soc > 80)
	{
		HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_02_GPIO_Port, LED_02_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_03_GPIO_Port, LED_03_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_04_GPIO_Port, LED_04_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_05_GPIO_Port, LED_05_Pin);
		HAL_Delay(200);

	}

}
void UnlockEeprom(void)

{

   while ((FLASH->SR & FLASH_SR_BSY) != 0) /* Wait for FLASH to be free */

   {

/*   insert timeout test */

   }

   if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* If PELOCK is locked */

   {

      FLASH->PEKEYR = FLASH_PEKEY1; /* Unlock PELOCK */

      FLASH->PEKEYR = FLASH_PEKEY2;

   }
   FLASH->PECR = FLASH->PECR | (FLASH_PECR_ERRIE | FLASH_PECR_EOPIE); /* enable flash interrupts */

}
/**
* Brief This function programs a word of data EEPROM.

* The ERASE bit and DATA bit are cleared in PECR at the beginning

* words are automatically erased if required before programming

* Param addr is the 32-bit EEPROM address to program, data is the 32 bit word to program

* Retval None
*/

void EepromProgram(uint32_t addr, uint32_t ee_data)

{

/* NOTE: The EEPROM must be unlocked and the flash interrupts must have been enabled prior to calling this function.*/

   *(uint32_t *)(addr) = ee_data; /* write data to EEPROM */

   __WFI();

   if (*(uint32_t *)(addr) != ee_data)

   {

      //error |= ERROR_PROG_32B_WORD;

   }
}
/* Lock the EEPROM: */
void LockEeprom(void)

{

   while ((FLASH->SR & FLASH_SR_BSY) != 0) /* Wait for FLASH to be free */

   {

/*   insert timeout test */

   }

   FLASH->PECR = FLASH->PECR & ~(FLASH_PECR_ERRIE | FLASH_PECR_EOPIE); /* disable flash interrupts */

   FLASH->PECR = FLASH->PECR | FLASH_PECR_PELOCK; /* Lock memory with PELOCK */

}


void Write_Registers_to_Eeprom(void)
{
uint32_t i;

uint32_t* p_tFlashRegs;
   i = 0;
   p_tFlashRegs = (uint32_t*)&data_to_write;   /* Point to data to write */

   UnlockEeprom(); /* Unlock the EEPROM and enable flash interrupts */

   FLASH->PECR = FLASH->PECR & ~(FLASH_PECR_ERASE | FLASH_PECR_DATA); /* Reset the ERASE and DATA bits in the FLASH_PECR register to disable any residual erase */
   /* Put the next line in a loop if sequential bits to be written with i as loop counter */
   EepromProgram(DATA_EEPROM_BASE_ADDR + (4 * i), *(p_tFlashRegs + i)); /* Increase eeprom address by 4 for each word to write.  */
   LockEeprom(); /* Lock the EEPROM */

}


void Write16(uint8_t Register, uint16_t Value)
{
	uint8_t addr[2];
	addr[0] = (Value >> 8) & 0xff;  // upper byte
	addr[1] = (Value >> 0) & 0xff; // lower byte

	HAL_I2C_Mem_Write(&hi2c1, INA220_ADDR, Register, 1, (uint8_t*)addr, 2, 1000);
}
void ResetINA()
{
	uint8_t addr[2];
	addr[0] = 0xB9;  // upper byte
	addr[1] = 0x9F; // lower byte
	HAL_I2C_Mem_Write(&hi2c1, INA220_ADDR, INA219_CONFIG, 1, addr, 2, 1000);
	HAL_Delay(1);
}

uint16_t Read16(uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(&hi2c1, INA220_ADDR, Register, 1, Value, 2, 1000);

	return ((Value[0] << 8) | Value[1]);
}

uint16_t INA219_ReadBusVoltage()
{

	uint16_t result = Read16(INA219_REG_BUSVOLTAGE);

	if(Saved == 0 && ((result >> 3  ) * 4) < 8.3){

		Saved = 1;

	}
	//return result;
	return ((result >> 3  ) * 4);

}

uint16_t INA219_ReadShuntVolage()
{
	uint16_t result = Read16(INA219_REG_SHUNTVOLTAGE);
	uint8_t signal = result >> 15;

	if (signal > 0){

		status_charger = 1;
		CurrentDraw = (~result) +1; // 2' complement


	}else {

		status_charger = 0;
		CurrentDraw  = result;
	}

	return result;
	//return (result * 0.01 ); // 0.01R Shunt
}

void Check_INA (){


		  bus_voltage =  INA219_ReadBusVoltage();
		  shunt_voltage =  INA219_ReadShuntVolage();
		  updatedValues ++;



}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim21);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, StartLED, osPriorityNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of Button */
  osThreadDef(Button, StartButton, osPriorityIdle, 0, 128);
  ButtonHandle = osThreadCreate(osThread(Button), NULL);

  /* definition and creation of INA220 */
  osThreadDef(INA220, StartINA220, osPriorityIdle, 0, 128);
  INA220Handle = osThreadCreate(osThread(INA220), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim21.Init.Prescaler = 8000-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 60000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_01_Pin|LED_02_Pin|LED_03_Pin|LED_04_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_05_Pin|battery_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_01_Pin LED_02_Pin LED_03_Pin LED_04_Pin */
  GPIO_InitStruct.Pin = LED_01_Pin|LED_02_Pin|LED_03_Pin|LED_04_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_05_Pin battery_Pin */
  GPIO_InitStruct.Pin = LED_05_Pin|battery_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Btn_Pin VCC_Bat_Pin Charger_Pin */
  GPIO_InitStruct.Pin = Btn_Pin|VCC_Bat_Pin|Charger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : _5V_Out_Pin */
  GPIO_InitStruct.Pin = _5V_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(_5V_Out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {

	  if(status_charger == 1){

		  ampereHour -= (CurrentDraw * (10 / 3600000.0)); // Calculate Ah value if the battery is discharging

	  } else {

		  ampereHour += (CurrentDraw * (10 / 3600000.0)); // Calculate Ah value if the battery is charging
	  }

	  soc = (ampereHour / batteryCap) * 100; // Calculate SOC in %

  }  if(htim->Instance == TIM21)
  {

		 char str[100];

		 int CurrentInt = ampereHour;

		 sprintf (str, "Voltage = %d", bus_voltage);
		 HAL_UART_Transmit(&hlpuart1, (uint8_t *)str, strlen(str), 1);

		 sprintf (str, " Capacity  = %d", CurrentInt);

		 HAL_UART_Transmit(&hlpuart1, (uint8_t *)str, strlen(str), 1);

		 sprintf (str, " Current = %d\n", CurrentDraw);

		 HAL_UART_Transmit(&hlpuart1, (uint8_t *)str, strlen(str), 1);
  }
  if(htim->Instance == TIM22)
  {
	  //HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_SET);
	  //osDelay(1);
	  //HAL_GPIO_WritePin(LED_01_GPIO_Port, LED_01_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_TogglePin(LED_01_GPIO_Port, LED_01_Pin);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
HAL_GPIO_WritePin(battery_GPIO_Port, battery_Pin, GPIO_PIN_SET);


  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLED */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED */
void StartLED(void const * argument)
{
  /* USER CODE BEGIN StartLED */


  /* Infinite loop */
  for(;;)
  {

	  if (updatedValues > 20 && setInitialCapacity == 0 && status_charger == 0 && CurrentDraw > 200){

		  	ampereHour = lookup(bus_voltage);
		  	HAL_TIM_Base_Start_IT(&htim2);
			setInitialCapacity = 1;

	  }else if (updatedValues > 20 && status_charger == 0 && CurrentDraw < 200 && setInitialCapacity == 0){

		  	ampereHour = batteryCap;
		  	HAL_TIM_Base_Start_IT(&htim2);
			setInitialCapacity = 1;
	  }

	  if (!ButtonState){

		  DisplayLevel();
		  clear_indicator_leds();


	  }
	  else if (status_charger == 0 && CurrentDraw < 300) {

		  osDelay(1000);

		 if (status_charger == 0) {

			  set_indicator_leds_blink();

		  }

	  }else if (status_charger == 0 && CurrentDraw > 300){
		  set_indicator_leds_blink();
	  }

	  else if (status_charger == 1){
	  		  clear_indicator_leds();
	  	  }

    osDelay(1);
  }
  /* USER CODE END StartLED */
}

/* USER CODE BEGIN Header_StartButton */
/**
* @brief Function implementing the Button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButton */
void StartButton(void const * argument)
{
  /* USER CODE BEGIN StartButton */
  /* Infinite loop */
  for(;;)
  {

	ButtonState = HAL_GPIO_ReadPin(Btn_GPIO_Port, Btn_Pin);

	ChargerState = HAL_GPIO_ReadPin(Charger_GPIO_Port, Charger_Pin);

    osDelay(1);
  }
  /* USER CODE END StartButton */
}

/* USER CODE BEGIN Header_StartINA220 */
/**
* @brief Function implementing the INA220 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINA220 */
void StartINA220(void const * argument)
{
  /* USER CODE BEGIN StartINA220 */

  /* Infinite loop */
  for(;;)
  {
	 Check_INA();
	 Real_capacity = lookup(bus_voltage);
    osDelay(50);
  }
  /* USER CODE END StartINA220 */
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

#ifdef  USE_FULL_ASSERT
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
