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
#include <stdbool.h>

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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool received = false;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	received = true;
}

void sendMessage8(int DLC, int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7)
{
	received = false;

	  TxHeader.DLC = DLC;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.StdId = 0x6B; // ID

	  TxData[0] = d0;
	  TxData[1] = d1;
	  TxData[2] = d2;
	  TxData[3] = d3;
	  TxData[4] = d4;
	  TxData[5] = d5;
	  TxData[6] = d6;
	  TxData[7] = d7;

	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) while(true)
	  {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_Delay(100);
	  }
	  HAL_Delay(100);
}

void sendMessage7(int DLC, int d0, int d1, int d2, int d3, int d4, int d5, int d6)
{
	sendMessage8(DLC, d0, d1, d2, d3, d4, d5, d6, 0);
}

void sendMessage6(int DLC, int d0, int d1, int d2, int d3, int d4, int d5)
{
	sendMessage7(DLC, d0, d1, d2, d3, d4, d5, 0);
}

void sendMessage5(int DLC, int d0, int d1, int d2, int d3, int d4)
{
	sendMessage6(DLC, d0, d1, d2, d3, d4, 0);
}

void sendMessage4(int DLC, int d0, int d1, int d2, int d3)
{
	sendMessage5(DLC, d0, d1, d2, d3, 0);
}

void sendMessage3(int DLC, int d0, int d1, int d2)
{
	sendMessage4(DLC, d0, d1, d2, 0);
}

void sendMessage2(int DLC, int d0, int d1)
{
	sendMessage3(DLC, d0, d1, 0);
}

void sendMessage1(int DLC, int d0)
{
	sendMessage2(DLC, d0, 0);
}

void sendMessage0(int DLC)
{
	sendMessage1(DLC, 0);
}

void waitForMessage()
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	while(!received)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
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
  MX_GPIO_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_Delay(30);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);


	sendMessage1(1, 0xFF);//CCP_FLASH_READ   INFO_MODULE_NAME, get controller model
	waitForMessage();
	int error = RxData[0];

	sendMessage3(3, 0xF2, 64, 8);//CCP_FLASH_READ   INFO_MODULE_NAME, get controller model
	waitForMessage();
	char model[8];
	for(int i = 0; i < 8; i++) model[i] = (char)RxData[i];



	sendMessage3(3, 0xF2, 83, 2);//CCP_FLASH_READ   INFO_SOFTWARE_VER, get controller software version
	waitForMessage();
	uint8_t version[2];
	for(int i = 0; i < 2; i++) model[i] = RxData[i];



	sendMessage3(3, 0xF2, 4, 1);//CCP_FLASH_READ   CAL_TPS_DEAD_ZONE_LOW, get throttle low dead zone
	waitForMessage();
	int TLDeadZone = RxData[0]/2;



	sendMessage3(3, 0xF2, 5, 1);//CCP_FLASH_READ   CAL_TPS_DEAD_ZONE_HIGH, get throttle high dead zone
	waitForMessage();
	int THDeadZone = RxData[0]/2;



	sendMessage3(3, 0xF2, 38, 1);//CCP_FLASH_READ   CAL_BRAKE_DEAD_ZONE_LOW, get brake low dead zone
	waitForMessage();
	int BLDeadZone = RxData[0];



	sendMessage3(3, 0xF2, 39, 1);//CCP_FLASH_READ   CAL_BRAKE_DEAD_ZONE_HIGH, get brake high dead zone
	waitForMessage();
	int BHDeadZone = RxData[0];



	sendMessage1(1, 0x1B);//CCP_A2D_BATCH_READ1, BrakeA/D  TPS A/D  Operation Voltage A/D  Vs A/D  B+ A/D
	waitForMessage();
	double BrakeAD = RxData[0]/51.0;
	double TPSAD = RxData[1]/51.0;
	double OpVAD = RxData[2]/2.05;
	double VsAD = RxData[3]/25.0;
	double BPAD = RxData[4]/2.05;



	sendMessage1(1, 0x1A);//CCP_A2D_BATCH_READ2, la A/D  lb A/D  lc A/D Va A/D  Vb A/D  Vc A/D
	waitForMessage();
	int laAD = RxData[0];
	int lbAD = RxData[1];
	int lcAD = RxData[2];
	double VaAD = RxData[3]/2.05;
	double VbAD = RxData[4]/2.05;
	double VcAD = RxData[5]/2.05;



	sendMessage1(1, 0x33);//CCP_MONITOR1, PWM  enable motor rotation  mototr temp  controller temp  hot end MOSFET  cold end MOSFET
	waitForMessage();
	int PWM = RxData[0];
	bool motorOn = RxData[1] == 1;
	int motorTemp = RxData[2] - 80;
	int motorTempF = motorTemp * 9/5 + 32;
	bool motorTempConnected = motorTemp != 0xFF;
	int ContTemp = RxData[3];
	int ContTempHighMOSFET = RxData[4];
	int ContTempLowMOSFET = RxData[5];



	sendMessage1(1, 0x37);//CCP_MONITOR2, RPM  present current accounts for percent of the rated current of controller
	waitForMessage();
	int RPM = RxData[0] * 256;
	RPM += RxData[1];
	int PCAFPOTRCOC = RxData[2];



	sendMessage2(2, 0x42, 0);//COM_SW_ACC, Throttle switch
	waitForMessage();
	bool throttleSwitch = RxData[0] == 1;



	sendMessage2(2, 0x43, 0);//COM_SW_BRK, Brake switch
	waitForMessage();
	bool BrakeSwitch = RxData[0] == 1;



	sendMessage2(2, 0x44, 0);//COM_SW_REV, Reverse switch
	waitForMessage();
	bool ReverseSwitch = RxData[0] == 1;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

   canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
   canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
   canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   canfilterconfig.FilterIdHigh = 0x73<<5;
   canfilterconfig.FilterIdLow = 0;
   canfilterconfig.FilterMaskIdHigh = 0x73<<5;
   canfilterconfig.FilterMaskIdLow = 0x0000;
   canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
   canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
   canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)

   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
