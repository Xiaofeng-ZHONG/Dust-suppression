/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
		<read me first> Xiaofeng Zhong 2022.7.10
		SPI_CS          PA4					
		Pump  					PB6
		LED2            PC13
		note 1: set --> Target --> choose "use MicroLIB" for using printf().
		note 2: first use, E18 Mac address should be write to W25Q32.
		
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "cJSON.h"  
#include "assert.h"
#include "math.h"
#include "W25Qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_rebuffsize  400					// STM32 UART1 rebuffsize
#define E18_number      3					// number of E18 chips

#define addr_for_software_params  0x00   //0~0x1000 is used for storging software params (W25Q32)
#define addr_for_SV_state 0x1000   //0x1000~0x11000 is used for storging pump_state
#define addr_for_E18_short_addr 0x11000   //0x11000~0x31000 is used for storging software params (W25Q32)
#define W25Qxx_addr_for_Mac   0x31000   //0x31000~0xB1000  write and read begin at this address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
struct item{
int	report_data;					//1: report data   0:don't report data
int	Pump_state;             //1:run 0:stop
int	SV_state;								//1:open 0:close
int	MAX_Moisture;      //if biger than this value,the solenoid valve and pump will stop
int	Min_Moisture;      //if smaller than this value,the solenoid valve and pump will run
int	interval_time;    	//delay=interval_time*(100ms)
int	auto_control;         //1: Auto  0: Manual
int	E18_mac_cnt;
int ADDR_cnt;
}item1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void NBIoT_Init(void); //NB-IoT initialization and subscrib to topic "/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/service/property/set/",every time it start
void W25Q32_Init(void);
void Params_Init(void);
int E18_Init(void);
int Get_ADDR(int Serial_Num);  // return -1:failed , addr
void Get_ALL_ADDR(void);
int Get_ADC(void);			// return -1:failed , adc
int BF(char* str, char* sub); //string matching
int SV_operate(int SV_state,int ADDR);  //return 1:ok, -1:failed
int Pump_operate(int Pump_state,int Serial_Num);   // return 1:on, 0:off
void MY_UART1_RxCpltCallback(void);
int fputc(int ch, FILE *f);
void Moisture_Get_and_Upload(void);  //Get Moisture from E18 and Report data through EC-01
void Record_Mac_cnt(void);
void Record_Params(void);
void Record_SV_state(int SV_state,int Serial_Num);
int Read_SV_state(void);
int Read_ADDR(int Serial_Num);
int Write_ADDR(uint8_t buffer[],int Serial_Num);
void Array_sort(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t W25Q32_read_buff[14];
uint8_t usart1_rebuff[MAX_rebuffsize];
uint8_t usart1_copy[MAX_rebuffsize];
uint16_t usart1_Rx_cnt=0;
uint8_t RecFlag=0;
uint8_t JSON_rebuff[200];
uint8_t usart6_rebuff[20];
uint8_t usart6_rebuf;
uint16_t usart6_Rx_cnt=0;
uint8_t usart2_rebuff[20];
uint8_t usart2_rebuf;
uint16_t usart2_Rx_cnt=0;
uint16_t detect_cnt=0;
volatile int adc_list[5];
uint8_t led_state=0;
float EC_sent_buffer[10][4];
volatile int EC_sent_cnt=1;
unsigned long mac_num=E18_number;
volatile uint8_t L2[7]={0xFE,0x04,0x23,0x00,0x00,0x02,0xFF};//Hex command for getting adc vale
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);
	W25Q32_Init();
	Params_Init();
	E18_Init();
	NBIoT_Init();
	Get_ALL_ADDR();
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);  
  HAL_UART_Receive_DMA(&huart1,usart1_rebuff,MAX_rebuffsize);
	MX_IWDG_Init();	 // note!!!, MX_IWDG_Init() must follow NBIoT_Init and other user code!!!  and  MX_DMA_Init() must before MX_USART1_UART_Init()
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		if (RecFlag==1){												//Judge DMA UART1 receive flag,
				MY_UART1_RxCpltCallback();					//deal with message received from UART1
				Record_Params();										//record user settings in W25Q32(spi flash)
				RecFlag=0;													//reset flag
		}
		if (detect_cnt>=(item1.interval_time*50)){  							//soil moisture detect interval,minimum scale:0.1s
				detect_cnt=0;																					//delay count reset
				//printf("E18_mac_cnt:%d\r\n",item1.E18_mac_cnt);  		//show the current number of E18 by which soil moisture is being detected
				Moisture_Get_and_Upload();														//show the current number of E18 by which soil moisture is being detected
				Record_Mac_cnt();																			//show the current number of E18 by which soil moisture is being detected
				if(item1.E18_mac_cnt>=(mac_num-1)){
							item1.E18_mac_cnt=0;}
				else{item1.E18_mac_cnt++;}														//Polling statement
		}else{detect_cnt++;HAL_Delay(1);}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void NBIoT_Init(void){
	printf("NB-IoT init .....\r\n"); //check whether the startup is ok
	printf("stpe0\r\n"); 
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"ATE0\r\n",6,0xFFFF);//	Do not send back instructions
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe1\r\n"); //check whether the startup is ok
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT\r\n",4,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe2\r\n");//show SIM card number
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECICCID\r\n",12,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 64, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){printf("%s\r\n",usart1_rebuff);memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe3\r\n");//Turn off flight mode
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+CFUN=1\r\n",11,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe4\r\n");//PS attachment
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+CGATT=1\r\n",12,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe5\r\n");//show signal quality
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+CSQ\r\n",8,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 64, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){printf("%s\r\n",usart1_rebuff);memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe6\r\n");//active PDP Context
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+CGDCONT=1,IP,CMNET\r\n",23,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe7\r\n");//keep alive every 60 seconds
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTCFG=\"keepalive\",0,60\r\n",64,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe8\r\n");//Set the cloud platform to Alibaba cloud and the data type to JSON
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTCFG=\"cloud\",0,2,1\r\n",27,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe9\r\n");//Set MQTT account
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTCFG=\"aliauth\",0,\"gc6pOMrLMi8\",\"NB-IoT_EC-01-Kit\",\"696de0a6ad9bb5b66f8140e8aac5dad9\"\r\n",92,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 6, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe10\r\n");//IoT platform address and port
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTOPEN=0,\"gc6pOMrLMi8.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\r\n",72,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 64, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe11\r\n");//Create an mqtt client and register the device in the cloud
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTCONN=0,\"12345\"\r\n",23,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 64, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("stpe12\r\n");//Subscribe to Topics
	while(1){
	HAL_UART_Transmit(&huart1, (uint8_t *)"AT+ECMTSUB=0,1,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/service/property/set\",1\r\n",81,0xFFFF);
	HAL_UART_Receive(&huart1, (void *)&usart1_rebuff, 64, 1000);
		if(BF((char*)usart1_rebuff, "OK")!=-1){memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));break;}
		memset(usart1_rebuff,0x00,sizeof(usart1_rebuff));
		HAL_Delay(200);
	}
	printf("NB-IoT Init OK.....\r\n");//NB-IoT Initialization succeeded
	//publish topics
	char str1[512];
	sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"auto_control\":{\"value\":%d},\"report_data\":{\"value\":%d},\"MAX_Moisture\":{\"value\":%d},\"Min_Moisture\":{\"value\":%d},\"interval_time\":{\"value\":%d},\"Pump\":{\"value\":%d}}}\"\r\n",item1.auto_control,item1.report_data,item1.MAX_Moisture,item1.Min_Moisture,item1.interval_time,item1.Pump_state);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),5000);while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	printf("publish topic OK....\r\n");
	
}


int BF(char* str, char* sub){//string matching
	assert(str&&sub);
	if (str == NULL || sub == NULL)
	{
		return -1;
	}
	int i = 0;
	int j = 0;
	int lenstr = strlen(str);
	int lensub = strlen(sub);
	while ((i < lenstr) && (j < lensub))
	{
		if (str[i] == sub[j])
		{
			i++;
			j++;
		}
		else
		{
			i = i - j + 1;
			j = 0;
		}
	}
	if (j >= lensub)
	{
		return i - j;
	}
	else
		return -1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //led  twinkle and IWDG feed
	if (htim->Instance == htim2.Instance) 
	{	HAL_IWDG_Refresh(&hiwdg);
		if (led_state){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
		}else{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);}
		led_state=!led_state;
	}
}


void Moisture_Get_and_Upload(void){
	int address;float adc;int Serial_Num=item1.E18_mac_cnt+1;int judge;
	if(item1.E18_mac_cnt==0){address=65535;}else{
	address=Read_ADDR(Serial_Num);}
	if (address==65529){
				int addr_value=Get_ADDR(Serial_Num);
				if (addr_value==-1){return;}
				else{
						L2[3]=(uint8_t)(addr_value/256);L2[4]=(uint8_t)(addr_value%256);
						uint8_t W25Q32_write_buff[2]={L2[3],L2[4]};
						Write_ADDR((unsigned char*)W25Q32_write_buff,Serial_Num);}			
		}else{L2[3]=(uint8_t)(address/256);L2[4]=(uint8_t)(address%256);}
		//printf("address:%d...",address);
		
		for(int j=0;j<5;j++){adc_list[j]=Get_ADC();}						//get adc value 5 times 
		Array_sort();																						//Array sort
		adc=(adc_list[1]+adc_list[2]+adc_list[3])/3;						//Eliminate the maximum and minimum values, then take the average value
		adc=(3300-adc);																					//Change negative correlation data into positive correlation data
		adc=(adc-1742)*100/(887);																//Using fuzzy mathematics to define humidity percentage
		
		if (adc>100){adc=100;}						//Set upper limit
		if (adc<0){adc=0;}								//Set lower limit
		
		judge=adc_list[0];
		memset((void*)&adc_list,0x00,sizeof(adc_list));
	if ((judge==-1)||(item1.auto_control==0)){
				int addr_value=Get_ADDR(Serial_Num);
				if (addr_value==-1){return;}
				else{
					L2[3]=(uint8_t)(addr_value/256);L2[4]=(uint8_t)(addr_value%256);
					uint8_t W25Q32_write_buff[2]={L2[3],L2[4]};
					Write_ADDR((unsigned char*)W25Q32_write_buff,Serial_Num);}
	}else{
		if ((adc>item1.MAX_Moisture)||(adc<item1.Min_Moisture)){
					if (adc>item1.MAX_Moisture){item1.SV_state=0;item1.Pump_state=0,SV_operate(item1.SV_state,address);item1.Pump_state=Pump_operate(item1.Pump_state,Serial_Num);}
					if (adc<item1.Min_Moisture){item1.SV_state=1;item1.Pump_state=1,SV_operate(item1.SV_state,address);item1.Pump_state=Pump_operate(item1.Pump_state,Serial_Num);}
				if (item1.report_data==1){
							if(item1.interval_time>10){
									char str1[300];
									sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"SM%d\":{\"value\":%.1f},\"Serial_Num\":{\"value\":%d},\"SV%d\":{\"value\":%d},\"Pump\":{\"value\":%d}}}\"\r\n",Serial_Num,adc,Serial_Num,Serial_Num,item1.SV_state,item1.Pump_state);
									HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),5000);
									printf("publish topic OK....\r\n");
							}else	{
										EC_sent_buffer[EC_sent_cnt-1][0]=Serial_Num;EC_sent_buffer[EC_sent_cnt-1][1]=adc;EC_sent_buffer[EC_sent_cnt-1][2]=item1.SV_state;EC_sent_buffer[EC_sent_cnt-1][3]=item1.Pump_state;
										if (EC_sent_cnt%((int)(10/item1.interval_time))==0){
													char str1[1024]="AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{";
													int count=0;
													for (int i=0;i<(int)(10/item1.interval_time);i++){
															char str2[150];
															sprintf(str2,"\"SM%d\":{\"value\":%.1f},\"SV%d\":{\"value\":%d},",(int)EC_sent_buffer[i][0],EC_sent_buffer[i][1],(int)EC_sent_buffer[i][0],(int)EC_sent_buffer[i][2]);
															sprintf(str1,"%s%s",str1,str2);
															if (EC_sent_buffer[i][3]==1){count=1;}
													}
													char str3[20];
													sprintf(str3,"\"Pump\":{\"value\":%d}",count);
													sprintf(str1,"%s%s",str1,str3);
													int len =strlen((void *)&str1);
													str1[len]='}';str1[len+1]='}';str1[len+2]='\"';str1[len+3]='\r';str1[len+4]='\n';
													HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),5000);
													//HAL_UART_Transmit(&huart6,(uint8_t*)str1,strlen(str1),5000);while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
													printf("publish topic OK....\r\n");
													EC_sent_cnt=0;
										}EC_sent_cnt++;
							}
				}
		}else{
								if (item1.report_data==1){
											if(item1.interval_time>10){
														char str1[300];
														sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"SM%d\":{\"value\":%.1f}}}\"\r\n",Serial_Num,adc);
														HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),5000);
														printf("publish topic OK....\r\n");
											}else	{
														EC_sent_buffer[EC_sent_cnt-1][0]=Serial_Num;EC_sent_buffer[EC_sent_cnt-1][1]=adc;
														if (EC_sent_cnt%((int)(10/item1.interval_time))==0){
																	char str1[1024]="AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{";
																	for (int i=0;i<(int)(10/item1.interval_time);i++){
																			char str2[150];
																			sprintf(str2,"\"SM%d\":{\"value\":%.1f},",(int)EC_sent_buffer[i][0],EC_sent_buffer[i][1]);
																			sprintf(str1,"%s%s",str1,str2);
																	}
																	int len =strlen((void *)&str1);
																	str1[len]='}';str1[len+1]='}';str1[len+2]='\"';str1[len+3]='\r';str1[len+4]='\n';
																	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),5000);
																	//HAL_UART_Transmit(&huart6,(uint8_t*)str1,strlen(str1),5000);while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
																	printf("publish topic OK....\r\n");
																	EC_sent_cnt=0;
														}EC_sent_cnt++;
													}
					}
			
			
			
			}
	}
}


int SV_operate(int SV_state,int ADDR){
		int timeout=8;
		uint8_t L1[8]={0xFD,0x05,0x20,0xFF,0xFF,0x03,0x00,0xFF}; //gpio output
		uint8_t L2[8]={0xFD,0x05,0x21,0xFF,0xFF,0x03,0x01,0xFF}; //gpio output level
		while(timeout--){
						L1[3]=(uint8_t)(ADDR/256);L1[4]=(uint8_t)(ADDR%256);
						HAL_UART_Transmit(&huart2,(void *)&L1,8,1000);
						HAL_UART_Receive(&huart2, (void *)&usart2_rebuff, 4,1000);
							if((usart2_rebuff[0] == 0xFA)&&(usart2_rebuff[1] == 0x20))
									{
										memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));break;}
							memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));
				}
		while(timeout--){
						if(SV_state==1){
								L2[3]=(uint8_t)(ADDR/256);L2[4]=(uint8_t)(ADDR%256);L2[6]=0x01;}
						else if (SV_state==0){
								L2[3]=(uint8_t)(ADDR/256);L2[4]=(uint8_t)(ADDR%256);L2[6]=0x00;}
						
						HAL_UART_Transmit(&huart2,(void *)&L2,8,1000);
						HAL_UART_Receive(&huart2, (void *)&usart2_rebuff, 4,1000);
							if((usart2_rebuff[0] == 0xFA)&&(usart2_rebuff[1] == 0x21))
									{
										memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));break;}
							memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));
				}
		if (timeout<0){ printf("SV_operate falied !!!\r\n");return -1;}else{return 1;}
}

int Pump_operate(int Pump_state,int Serial_Num){
		int pump_on;int n;
		if (item1.auto_control==0){
				if(Pump_state==1){pump_on=1;HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);}
				else if(Pump_state==0){pump_on=0;HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);}
		}else if (item1.auto_control==1){
		Record_SV_state(Pump_state,Serial_Num);
		n=Read_SV_state();
		if (n==1){    // only when every pump state is off ,the pump will off.
					pump_on=1;HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);}
		else if(n==-1){ 
				 pump_on=0;HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);}
		}
		return pump_on;
}

int Get_ADDR(int Serial_Num){
	int timeout=8;int addr=-1;uint8_t L1[12]={0xFE,0x09,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF};
	W25QXX_Read((uint8_t*)W25Q32_read_buff,W25Qxx_addr_for_Mac+(Serial_Num-1)*8,8);
	for(int j=0;j<8;j++){L1[j+3]=W25Q32_read_buff[j];}
	memset(W25Q32_read_buff,0x00,sizeof(W25Q32_read_buff));
	while(timeout--){	
									HAL_UART_Transmit(&huart2,(void *)&L1,12,0x1000);while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
									HAL_UART_Receive(&huart2, (void *)&usart2_rebuff, 4,500);
										if(usart2_rebuff[0] == 0xFB)
												{addr=usart2_rebuff[1]*256+usart2_rebuff[2];memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));break;}
												memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));
							}
	if (timeout<0){ printf("get short_address falied !!!\r\n");return -1;}else{return addr;}

}
int Read_ADDR(int Serial_Num){
	W25QXX_Read((uint8_t*)W25Q32_read_buff,addr_for_E18_short_addr+(Serial_Num-1)*8,2);
	int addr=W25Q32_read_buff[0]*256+W25Q32_read_buff[1];
	memset(W25Q32_read_buff,0x00,sizeof(W25Q32_read_buff));
	return addr;
}
int Write_ADDR(uint8_t buffer[],int Serial_Num){
			W25QXX_Write_Enable();
			W25QXX_Write((unsigned char*)buffer,addr_for_E18_short_addr+(Serial_Num-1)*8,2);
			W25QXX_Write_Disable();
return 1;
}
void Get_ALL_ADDR(void){
	int addr=-1;
	for (int k=item1.ADDR_cnt;k<mac_num;k++){
	printf("get short_address  No.%d...\r\n",k+1);
	addr=Get_ADDR(k+1);
	if (addr==-1){
			uint8_t W25Q32_write_buff[2]={0xFF,0xF9};
			Write_ADDR((unsigned char*)W25Q32_write_buff,k+1);
			}
	else{
			uint8_t W25Q32_write_buff[2]={(uint8_t)(addr/256),(uint8_t)(addr%256)};
			Write_ADDR((unsigned char*)W25Q32_write_buff,k+1);
			} 
		W25QXX_Write_Enable();
		uint8_t num[2]= {k/256,k%256};
		W25QXX_Write((uint8_t*)num,12,2);
		W25QXX_Write_Disable();
		HAL_Delay(400);
	}
	printf("get all ADDR  OK  !!! \r\n");
}
int Get_ADC(void){
	int timeout=8;int adc;
	while(timeout--){
	HAL_UART_Transmit(&huart2,(void *)&L2,7,0x1000);while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
	HAL_UART_Receive(&huart2, (void *)&usart2_rebuff, 6,1000);
		if((usart2_rebuff[0] == 0xFB)&&(usart2_rebuff[1] == 0x23))
				{
					adc=usart2_rebuff[4]*256+usart2_rebuff[5];
					memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));break;}
		memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));
					HAL_Delay(10);	
				}
		if (timeout<0){ printf("get short_address falied !!!\r\n");return -1;}else{return adc;}
}


void MY_UART1_RxCpltCallback(void){  
		usart1_copy[usart1_Rx_cnt] = '\0';
		usart1_Rx_cnt = 0;
		int cnt=BF((void *)&usart1_copy, "params");
		if(cnt!=-1){
		for (int j=cnt-2;j<strlen((void *)&usart1_copy);j++){usart1_copy[j-cnt+2]=usart1_copy[j];}
		for (int j=strlen((void *)&usart1_copy)-cnt+2;j<strlen((void *)&usart1_copy);j++){usart1_copy[j]=0x00;}
		usart1_copy[0]='{';
		int len=strlen((void *)&usart1_copy);
		cnt=BF((void *)&usart1_copy, "version");
		for (int j=cnt-1;j<len;j++){usart1_copy[j]=0x00;}	usart1_copy[cnt-2]='}';
		HAL_UART_Transmit(&huart6, (uint8_t *)usart1_copy,strlen((void *)&usart1_copy),3000); while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
		
//					
		cJSON* cjson = cJSON_Parse((void *)&usart1_copy);
		if(cjson == NULL)						
		{
					printf("\nJSON Parse failed...\n");
		}
		else 
		{	
					cJSON* item = cJSON_GetObjectItem(cjson,"params");
						printf("\nJSON Parse OK...\n");
					if(BF((void *)&usart1_copy, "auto_control")!=-1){item1.auto_control=cJSON_GetObjectItem(item,"auto_control")->valueint;}
					if(BF((void *)&usart1_copy, "report_data")!=-1){item1.report_data=cJSON_GetObjectItem(item,"report_data")->valueint;}
					if(BF((void *)&usart1_copy, "MAX_Moisture")!=-1){item1.MAX_Moisture=cJSON_GetObjectItem(item,"MAX_Moisture")->valueint;}
					if(BF((void *)&usart1_copy, "Min_Moisture")!=-1){item1.Min_Moisture=cJSON_GetObjectItem(item,"Min_Moisture")->valueint;}
					if(BF((void *)&usart1_copy, "interval_time")!=-1){item1.interval_time=cJSON_GetObjectItem(item,"interval_time")->valueint;}
					
					char str1[256];
					sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"auto_control\":{\"value\":%d},\"report_data\":{\"value\":%d},\"MAX_Moisture\":{\"value\":%d},\"Min_Moisture\":{\"value\":%d},\"interval_time\":{\"value\":%d}}}\"\r\n",item1.auto_control,item1.report_data,item1.MAX_Moisture,item1.Min_Moisture,item1.interval_time);
					HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0xffff);while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
					HAL_UART_Transmit(&huart6, (uint8_t *)"publish topic OK....\r\n",22,1000);
					
					//

					int num1;int Serial_Num=-1;
					char mach[7];
					num1=BF((void *)&usart1_copy, (void *)"SV");
					if(num1!=-1){
					for (int k=0;k<7;k++){ mach[k]=usart1_copy[num1+k];}
					sscanf(mach, "SV%d", &Serial_Num);
					sprintf(str1,"SV%d",Serial_Num);
					item1.SV_state=cJSON_GetObjectItem(item,str1)->valueint;
					//
					int addr;
					if(Serial_Num==1){addr=65535;}
					else{addr=Read_ADDR(Serial_Num);}
					if (SV_operate(item1.SV_state,addr)==-1){;}
					else {
							sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"SV%d\":{\"value\":%d}}}\"\r\n",Serial_Num,item1.SV_state);
							HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0xffff);while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
							HAL_UART_Transmit(&huart6, (uint8_t *)"publish topic OK....\r\n",22,1000);
					}}
					//	
					

					if(BF((void *)&usart1_copy, "Pump")!=-1){
					item1.Pump_state=cJSON_GetObjectItem(item,"Pump")->valueint;item1.Pump_state=Pump_operate(item1.Pump_state,Serial_Num);
					sprintf(str1,"AT+ECMTPUB=0,1,1,0,\"/sys/gc6pOMrLMi8/NB-IoT_EC-01-Kit/thing/event/property/post\",\"{\"params\":{\"Pump\":{\"value\":%d}}}\"\r\n",item1.Pump_state);
					HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0xffff);while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
					HAL_UART_Transmit(&huart6, (uint8_t *)"publish topic OK....\r\n",22,1000);
					}
				//						
		}cJSON_Delete(cjson);
		memset(usart1_copy,0x00,sizeof(usart1_copy));
}}

int E18_Init(void){
	uint8_t L4[4]={0xFD,0x01,0x12,0xFF};
	int timeout=8;
	while(timeout--){
						HAL_UART_Transmit(&huart2,(void *)&L4,4,1000);
						HAL_UART_Receive(&huart2, (void *)&usart2_rebuff, 2,1000);
							if((usart2_rebuff[0] == 0xFA)&&(usart2_rebuff[1] == 0x12))
									{
										memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));break;}
							memset(usart2_rebuff,0x00,sizeof(usart2_rebuff));
				}
		if (timeout<0){ printf("E18 reset falied !!!\r\n");return -1;}else{printf("E18 reset OK !!!\r\n");return 1;}

}

void W25Q32_Init(void){
	printf("W25QXX Init :%d\n",W25QXX_Init());
	printf("W25QXX ID :%d\n",W25QXX_ReadID());
	for(int i=1;i<17;i++){
	W25QXX_Erase_Sector(i);}
}
int fputc(int ch, FILE *f){
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch,1, 0xFFFF);
    return ch;
}
void Params_Init(void){
	W25QXX_Read((uint8_t*)W25Q32_read_buff,0,14);
	//for(int i=0;i<12;i++){printf("%02x",W25Q32_read_buff[i]);}
	if ((W25Q32_read_buff[2]==0xFF)&&(W25Q32_read_buff[3]==0xFF)){
		item1.auto_control=1;			//1-bit, 1-yes,2-no
		item1.report_data=1;			//1-bit,1-yes,2-no
		item1.interval_time=100;  //2-bit,time=100*100ms=10s
		item1.MAX_Moisture=90;	//(%)
		item1.Min_Moisture=50;	//(%)
		item1.Pump_state=0;				//1-bit,1-run,2-stop
		item1.SV_state=0;					//1-bit,1-open,2-close
		item1.E18_mac_cnt=0;			//2-bit
		item1.ADDR_cnt=0;			//2-bit
		Record_Params();
	}else{
		item1.auto_control= W25Q32_read_buff[0];
		item1.report_data= W25Q32_read_buff[1];	
		item1.interval_time=W25Q32_read_buff[2]*256+W25Q32_read_buff[3];
		item1.MAX_Moisture=W25Q32_read_buff[4]*256+W25Q32_read_buff[5];
		item1.Min_Moisture=W25Q32_read_buff[6]*256+W25Q32_read_buff[7];
		item1.Pump_state=0;				
		item1.SV_state=W25Q32_read_buff[9];			
		item1.E18_mac_cnt=W25Q32_read_buff[10]*256+W25Q32_read_buff[11];
		if ((W25Q32_read_buff[12]==0xFF)&&(W25Q32_read_buff[13]==0xFF)){item1.ADDR_cnt=0;}
		else{item1.ADDR_cnt=W25Q32_read_buff[12]*256+W25Q32_read_buff[13];}
		printf("read Params OK....\n");
	}
	memset(W25Q32_read_buff,0x00,sizeof(W25Q32_read_buff));
}

void Record_Mac_cnt(void){
		W25QXX_Write_Enable();
		uint8_t W25Q32_write_buff[2]={item1.E18_mac_cnt/256,item1.E18_mac_cnt%256};
		W25QXX_Write((uint8_t*)W25Q32_write_buff,10,2);
		W25QXX_Write_Disable();
}
void Record_Params(void){
		uint8_t W25Q32_write_buff[12]={item1.auto_control,item1.report_data,item1.interval_time/256,item1.interval_time%256,item1.MAX_Moisture/256,
		item1.MAX_Moisture%256,item1.Min_Moisture/265,item1.Min_Moisture%256,item1.Pump_state,item1.SV_state,item1.E18_mac_cnt/256,item1.E18_mac_cnt%256
		};
		W25QXX_Write_Enable();
		W25QXX_Write((uint8_t*)W25Q32_write_buff,0,12);
		W25QXX_Write_Disable();
}
void Record_SV_state(int SV_state,int Serial_Num){
		W25QXX_Write_Enable();
		unsigned char a[2]={0x00,0xff};
		if(SV_state==0){
				W25QXX_Write((void *)&a[1],addr_for_SV_state+Serial_Num-1,1);}// if poen ,write 0,if close write 1
		else if (SV_state==1){
				W25QXX_Write((void *)&a[0],addr_for_SV_state+Serial_Num-1,1);}// if poen ,write 0,if close write 1
		W25QXX_Write_Disable();
}
int Read_SV_state(void){
		unsigned char read_buf[1]={0xff};
		for(int k=0;k<E18_number;k++){
		W25QXX_Read((void *)&read_buf,addr_for_SV_state+k,1);
		if (read_buf[0]==0x00){return 1;}//if one more SV is opened ,the pump will run 
		}
	return -1;
}
void Array_sort(void){
    int i, j, temp, isSorted;
    for(i=0; i<5-1; i++){
        isSorted = 1;  
        for(j=0; j<5-1-i; j++){
            if(adc_list[j] > adc_list[j+1]){
                temp = adc_list[j];
                adc_list[j] = adc_list[j+1];
                adc_list[j+1] = temp;
                isSorted = 0;  
            }
        }
        if(isSorted) break; 
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
