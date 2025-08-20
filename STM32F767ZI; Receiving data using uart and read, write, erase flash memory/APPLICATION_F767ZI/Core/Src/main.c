/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t Address = 0x080E0000;
uint32_t TXT_Address = 0x080E0000;
uint32_t LED_Address = 0x080E0100;
uint8_t count;

char stored_sen[256];


int write(char *ptr);
static void InvalidateFlashRange(uint32_t start_addr, uint32_t length);
static void goto_bootloader(void);

void erase_flash(void);
void program_flash(uint8_t data, uint32_t address);
void readLED_flash(void);
void readTXT_flash(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	SCB->VTOR = 0x08100000;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();


  /* USER CODE BEGIN Init */
  uint32_t tickstartld = 0;
  uint32_t waitld = 500;

  GPIO_PinState Bt_state;

  uint32_t tick = 0;


  uint8_t buttonPressed = 0;
  uint32_t pressStartTick = 0;




  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  write("\nStarting application\n");

  readLED_flash();
  if(count == 0){
	  write("Blue led should blink");
  }
  else{
	  write("Red led should blink");
  }

  readTXT_flash();
  write("\nStored sentence: ");
  write(stored_sen);
  write("\r\n");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  tick = HAL_GetTick();

	  if ((tick - tickstartld) >= waitld)
	  {
	  	tickstartld = tick;

	  	if (count == 0) {
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);


	  	} else {
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	  		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	  	}
	  }

	  Bt_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  if (Bt_state == GPIO_PIN_SET) {
	  	if (!buttonPressed) {

	  		buttonPressed = 1;
	  		pressStartTick = tick;
	  	}
	  } else {
	  	if (buttonPressed) {

	  		buttonPressed = 0;
	  		if ((tick - pressStartTick) >= 3000) {
	  			goto_bootloader();
	  		} else {
	  			count = !count;
	  			erase_flash();
	  			uint32_t count_Address = TXT_Address;

	  			for (size_t i=0; i<strlen(stored_sen); i++) {
	  				program_flash((uint8_t)stored_sen[i], count_Address++);
	  			}
	  			program_flash(0x00, count_Address);

	  			InvalidateFlashRange(TXT_Address, (count_Address - TXT_Address) + 1);
	  			program_flash(count, LED_Address);

	  		}
	  	}
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int write(char *ptr){
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, strlen(ptr), HAL_MAX_DELAY);
	return strlen(ptr);
}


static void goto_bootloader(void)
{
	write("\nJumping to Bootloader");
	NVIC_SystemReset();
}

void erase_flash(void){
	FLASH_EraseInitTypeDef ERASE_STRUCT;
	ERASE_STRUCT.TypeErase = FLASH_TYPEERASE_SECTORS;
	ERASE_STRUCT.Banks = FLASH_BANK_1;
	ERASE_STRUCT.Sector = FLASH_SECTOR_11;
	ERASE_STRUCT.NbSectors = 1;
	ERASE_STRUCT.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t Sector_error;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&ERASE_STRUCT, &Sector_error);
	HAL_FLASH_Lock();

}

void program_flash(uint8_t data, uint32_t address){

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, data);
	HAL_FLASH_Lock();

}

void readLED_flash(void){

	uint8_t data = (uint8_t) (* ((volatile uint32_t*) LED_Address));

	if((data == 0) || (data == 1)){
		count = data;
	}
	else{
		count = 0;
	}

}

void readTXT_flash(void)
{
    const char *ptr = (const char *)TXT_Address;
    char *dst = stored_sen;

    while (1) {
        char c = *ptr++;
        *dst++ = c;
        if (c == '\0') {
            break;
        }
    }
}

static void InvalidateFlashRange(uint32_t start_addr, uint32_t length)
{

    uint32_t start = start_addr & ~31U;
    uint32_t end   = (start_addr + length + 31U) & ~31U;
    uint32_t size  = end - start;

    SCB_CleanInvalidateDCache_by_Addr((void*)start, (int32_t)size);
    SCB_InvalidateICache();
}

/* USER CODE END 4 */

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
