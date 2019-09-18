/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define sabitleme_toleransi 50
#define hizBoleni 1
#define INTERVAL 3000

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

typedef struct{

	uint8_t mod;
	uint8_t armco;
	uint16_t desired_angle;
	uint16_t valueJoyStick_X_1;
	uint16_t valueJoyStick_Y_1;
	uint16_t valueJoyStick_X_2;
	uint16_t valueJoyStick_Y_2;

}Master;

Master control = {0};

uint16_t adc_buffer[4];
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];//uint8_t TxData[8];
uint8_t RxData[8];//uint8_t RxData[8];
uint32_t TxMailbox;
uint8_t i=0;
char in[8];


uint32_t last_time;
uint16_t batt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map();
void uartPrintln();
void uartPrint();
int AutoMod();
int ManuelMod();
uint8_t createData(uint16_t bit16data);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);//eklendi

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
      {
          /* Filter configuration Error */
          Error_Handler();
      }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
      {
          /* Start Error */
          Error_Handler();
      }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
      {
          /* Notification Error */
          Error_Handler();
      }

  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  HAL_UART_Receive_IT(&huart1, in, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 char tx[64];
	 batt = RxData[0];
	 control.mod = 1;

	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
		 AutoMod();
	 }else{
		 ManuelMod();
	 }

	 //AutoMod();

	 HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "X_1: %d Y_1: %d X_2: %d Y_2: %d angle: %d arm: %d mod: %d\n", control.valueJoyStick_X_1,control.valueJoyStick_Y_1,control.valueJoyStick_X_2,control.valueJoyStick_Y_2,control.desired_angle,control.armco,control.mod), 500);


	 TxData[0] = createData(control.valueJoyStick_X_1);
	 TxData[1] = createData(control.valueJoyStick_Y_1);
	 TxData[2] = createData(control.valueJoyStick_X_2);
	 TxData[3] = createData(control.valueJoyStick_Y_2);
	 TxData[4] = createData(control.desired_angle);
	 TxData[5] = control.armco;
	 TxData[6] = control.mod;

	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

/*
	 char tx[10];
	 //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "Batt: %d\n", RxData[0]), 500);
	 HAL_UART_Transmit(&huart1, (uint8_t*)tx, 8, 500);
	 HAL_UART_Receive(&huart1, (uint8_t*)tx, 8, 1000);
	 //HAL_UART_Transmit(&huart1, (uint8_t*)tx, sprintf(tx, "X_1: %d Y_1: %d X_2: %d Y_2: %d\n", valueJoyStick_X_1,valueJoyStick_Y_1,valueJoyStick_X_2,valueJoyStick_Y_2), 500);
*/


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	 if(HAL_GetTick() - last_time > INTERVAL)
	   {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	      last_time = HAL_GetTick();
	   }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    if(HAL_GetTick() - last_time > INTERVAL)
    	   {
    		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	      last_time = HAL_GetTick();
    	   }
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void uartPrint(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}

void uartPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

int AutoMod(){


	control.valueJoyStick_X_1=1500;
    control.valueJoyStick_X_2=1500;
    control.valueJoyStick_Y_1=1500;
    control.valueJoyStick_Y_2=1500;
    control.desired_angle = 0;

	//HAL_UART_Receive(&huart1, (uint8_t *)in, 8, 10);
	//HAL_UART_Receive_IT(&huart1, in, 5);//eklendi
/*

	if(strcmp(in,"a")==0){
		control.valueJoyStick_X_1 = 1550;//sag
	}else if(strcmp(in,"b")==0){
		control.valueJoyStick_X_1 = 1450;//sol
	}else if(strcmp(in,"c")==0){
		control.valueJoyStick_Y_1 = 1700;//yukarı
	}else if(strcmp(in,"d")==0){
		control.valueJoyStick_Y_1 = 1300;//asagı
	}else if(strcmp(in,"s")==0){
		control.valueJoyStick_Y_2 = 1510;//ileri
	}else if(strcmp(in,"e")==0){
		control.valueJoyStick_Y_2 = 1500;//dur
	}else if(strcmp(in,"x")==0){
		i++;
		control.desired_angle = 1*i;//90 derece don//aci
		HAL_Delay(500);
	}else{
		 if(HAL_GetTick() - last_time > 3000)
		    {
			control.valueJoyStick_X_1=1500;
			control.valueJoyStick_X_2=1500;
			control.valueJoyStick_Y_1=1500;
			control.valueJoyStick_Y_2=1500;
			last_time = HAL_GetTick();

         	}


	}
	memset( in, 0, 8 );

	/*
	if(in[0] == 255){
		if(in[1] == 1){
			if(in[2] == 1){
			control.valueJoyStick_X_1 = 1600;//sag
			}else if(in[2] == 2){
			control.valueJoyStick_X_1 = 1400;//sol
			}else if(in[2] == 0){
			control.valueJoyStick_Y_2 = 1600;//ileri
			}
		}if(in[1] == 2){
			if(in[2] == 1){
			control.valueJoyStick_Y_1 = 1600;//yukarı
			}else if(in[2] == 2){
			control.valueJoyStick_Y_1 = 1400;//asagı
			}else if(in[2] == 0){
			control.valueJoyStick_Y_2 = 1600;//ileri
			}
		}if(in[1] == 3){
			if(in[2] == 1){
			i++;
			control.desired_angle = 1*i;//90 derece don
			}
		}if(in[1] == 4){
			if(in[2]==1){
			control.valueJoyStick_Y_2 = 1600;//ileri
			}else if(in[2]==2){
			control.valueJoyStick_Y_2 = 1500;//dur
			}
        }

	} */


	return 1;
}

int ManuelMod(){

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);

	control.valueJoyStick_X_1 = map(adc_buffer[0],580,3460,1050,1900);//sag x ekseni
    control.valueJoyStick_Y_1 = map(adc_buffer[1],250,3400,1050,1900);//sag y ekseni
	control.valueJoyStick_X_2 = map(adc_buffer[3],400,3515,1050,1900);//sol y ekseni
	control.valueJoyStick_Y_2 = map(adc_buffer[2],340,3650,1050,1900);//sol x ekseni
	control.desired_angle = 0; //son kod atıldı

	control.valueJoyStick_X_1 = 1500 + (control.valueJoyStick_X_1 - 1500)/hizBoleni;
	control.valueJoyStick_Y_1 = 1500 + (control.valueJoyStick_Y_1 - 1500)/hizBoleni;
	control.valueJoyStick_X_2 = 1500 + (control.valueJoyStick_X_2 - 1500)/hizBoleni;
    control.valueJoyStick_Y_2 = 1500 + (control.valueJoyStick_Y_2 - 1500)/hizBoleni;

    if (control.valueJoyStick_X_1 < 1500 + sabitleme_toleransi / hizBoleni && control.valueJoyStick_X_1 > 1500 - sabitleme_toleransi / hizBoleni)
		control.valueJoyStick_X_1 = 1500;
	if (control.valueJoyStick_Y_1 < 1500 + sabitleme_toleransi / hizBoleni && control.valueJoyStick_Y_1 > 1500 - sabitleme_toleransi / hizBoleni)
		control.valueJoyStick_Y_1 = 1500;
	if (control.valueJoyStick_X_2 < 1500 + sabitleme_toleransi / hizBoleni && control.valueJoyStick_X_2 > 1500 - sabitleme_toleransi / hizBoleni)
		control.valueJoyStick_X_2 = 1500;
    if (control.valueJoyStick_Y_2 < 1500 + sabitleme_toleransi / hizBoleni && control.valueJoyStick_Y_2 > 1500 - sabitleme_toleransi / hizBoleni)
		control.valueJoyStick_Y_2 = 1500;

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
		control.armco = 1;
    else
		control.armco = 0;
	return 1;
}

uint8_t createData(uint16_t bit16data)
{
	return (bit16data - 1000)/4;
}
/*
void USART1_IRQHandler(void)//eklendi
{
  HAL_UART_IRQHandler(&huart1);

  HAL_UART_Receive_IT(&huart1, in, 1);
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
