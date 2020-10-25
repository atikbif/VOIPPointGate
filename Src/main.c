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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "lwip.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "button_led.h"
#include "can_tx_stack.h"
#include "frame_stack.h"
#include "uart.h"
#include "can_cmd.h"
#include "eeprom.h"
#include "modbus.h"
#include "dyn_data.h"
#include "can_protocol.h"

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

/* USER CODE BEGIN PV */

extern CAN_HandleTypeDef hcan1;
//static CAN_RxHeaderTypeDef   RxHeader;
//static uint8_t               RxData[8];
extern tx_stack can1_tx_stack;
extern tx_stack can2_tx_stack;
extern uint32_t	can_caught_id;
extern uint16_t packet_tmr;

extern uint8_t net_addr;
extern uint8_t current_group;

uint16_t VirtAddVarTab[NB_OF_VAR]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

extern unsigned short holdReg[HoldingRegistersLimit];

uint16_t adc_data[3]={0,0,0};

extern uint16_t	supposed_point_cnt;	// предполагаемое число точек

#define		DI_BREAK_LIMIT		200
#define		DI_OPEN_LIMIT		800
#define		DI_CLOSED_LIMIT		2000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void initCANFilter() {
	CAN_FilterTypeDef  sFilterConfig;

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
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
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
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  HAL_FLASH_Unlock();
  EE_Init();
  EE_ReadVariable(VirtAddVarTab[0],  &holdReg[0]);
  if(holdReg[0]!=0x1235) {
	  EE_WriteVariable(VirtAddVarTab[0],0x1235);
	  holdReg[0] = 192;holdReg[1] = 168;holdReg[2] = 1;holdReg[3] = 2;
	  holdReg[4] = 255;holdReg[5] = 255;holdReg[6] = 255;holdReg[7] = 0;
	  holdReg[8] = 192;holdReg[9] = 168;holdReg[10] = 1;holdReg[11] = 1;
	  holdReg[12] = 1; // число предпол подкл точек
	  holdReg[13] = 1; // сетевой адрес
	  holdReg[14] = 0;	// контроль неисправности микрофона/динамиков откл
	  for(uint8_t i=0;i<NB_OF_VAR-1;i++) {
		  EE_WriteVariable(VirtAddVarTab[i+1],  holdReg[i]);
	  }
  }
  for(uint8_t i=0;i<NB_OF_VAR-1;i++) {
	  EE_ReadVariable(VirtAddVarTab[i+1],  &holdReg[i]);
  }

  current_group = net_addr = holdReg[13];
  supposed_point_cnt = holdReg[12];
  if(supposed_point_cnt>64 || supposed_point_cnt==0) {
	  holdReg[12]=supposed_point_cnt=1;
	  EE_WriteVariable(VirtAddVarTab[12+1],  supposed_point_cnt);
  }

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_3);
  LL_USART_EnableIT_RXNE(USART3);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &adc_data,3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  //TIM1->CCR2=32756;
  //TIM1->CCR3=10000;

  init_can_frames();
  init_can_tx_stack(&can1_tx_stack);
  init_can_tx_stack(&can2_tx_stack);

  initCANFilter();
  HAL_CAN_Start(&hcan1);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
  }
  HAL_CAN_Start(&hcan2);
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  init_groups();
  init_points();



  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1) check_can_rx(1);
	else check_can_rx(2);
	/*if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			toggle_second_led(GREEN);
		}
	}*/
}

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

  can_write_from_stack();
  packet_tmr++;
  if(packet_tmr>=500) {
	  packet_tmr=0;
	  can_caught_id = 0;
  }
  uart1_scan();

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
