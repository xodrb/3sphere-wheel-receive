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
#include "NRF24.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const int ADC_MAX = 4020;    // 실제 최대값
const int ADC_MIN = 0;
#define ADC_NEU 2010	//ADC 중간값 4020/2
#define ADC_DEAD_ZONE 200	//데드존 처리 100

#define ROTATION_CONST -0.5f    // 회전 상수

#define USE_ADC_FALLBACK 1  // 1이면 무선 없을 때 ADC로 대체

static uint32_t no_signal_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//함수 선언
void nrf24_receiver_setup(void);
int try_receive_nrf24(uint16_t *rawX, uint16_t *rawY, uint16_t *rawZ);
float NormalizeADC(int16_t raw);
uint16_t ToPWMus(float value);
void KiwiDrive(float vx, float vy, float omega);
void DebugUART(uint16_t rawX, uint16_t rawY, uint16_t rawZ);


uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // 수신 파이프 주소를 송신부와 동일하게 설정

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // nRF24 초기화 (수신기)
  nrf24_init();
  nrf24_receiver_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint16_t rawX = ADC_NEU, rawY = ADC_NEU, rawZ = ADC_NEU;
	  if(try_receive_nrf24(&rawX, &rawY, &rawZ)){
		  float vx = NormalizeADC((int16_t)rawX); // 부호 있어야함
		  float vy = NormalizeADC((int16_t)rawY);
		  float omega = NormalizeADC((int16_t)rawZ);

		  KiwiDrive(vx, vy, omega);
		  DebugUART(rawX, rawY, rawZ);
		  no_signal_count = 0;
	  }else{
		  // 통신 실패처리
		  no_signal_count++;
		  char buf[64];
		  int len = snprintf(buf, sizeof(buf),
				"No Signal: %lu\n", no_signal_count);

		  HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
	  }
	  HAL_Delay(20);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin_Pin|CE_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CSN_Pin_Pin CE_Pin_Pin */
  GPIO_InitStruct.Pin = CSN_Pin_Pin|CE_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void nrf24_receiver_setup(void){
    nrf24_defaults();                               //레지스터 기본값으로 리셋
    HAL_Delay(5);                                   //전원, spi안정화 대기(최소 4.5ms이상 필요)
    nrf24_stop_listen();
    nrf24_pwr_up();
    HAL_Delay(2);

    nrf24_flush_rx();
    nrf24_flush_tx();
    nrf24_clear_rx_dr();
    nrf24_clear_tx_ds();
    nrf24_clear_max_rt();

    nrf24_set_channel(40); //무선 채널 40설정
    nrf24_data_rate(_1mbps);
    nrf24_auto_ack_all(disable);
    nrf24_dpl(disable); //ack 비활성화
    nrf24_set_payload_size(6);
    nrf24_open_rx_pipe(0, rx_address);              //파이프 0에 수신주소 설정
    nrf24_listen();                                 //CE=high => 실제 수신 대기모드 진입
}


int try_receive_nrf24(uint16_t *rawX, uint16_t *rawY, uint16_t *rawZ){
    if (!nrf24_data_available()){
    	return 0;
    }

    uint8_t buf[6];
    nrf24_receive(buf, 6);

    *rawX = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    *rawY = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    *rawZ = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    nrf24_clear_rx_dr();

    while(nrf24_data_available()) {
        uint8_t dummy[6];
        nrf24_receive(dummy, 6);
        nrf24_clear_rx_dr();
    }
    return 1;
}


float NormalizeADC(int16_t raw){
    int16_t delta = raw - ADC_NEU;

    if(abs(delta) < ADC_DEAD_ZONE){
        return 0.0f;
    }

    if(delta > 0) {
        // CW 방향: 중간값~최대값 → 0~1
        return (float)delta / (float)(ADC_MAX - ADC_NEU);  // /2010
    } else {
        // CCW 방향: 최소값~중간값 → -1~0
        return (float)delta / (float)(ADC_NEU - ADC_MIN);  // /2010
    }
}

uint16_t ToPWMus(float v){
	if(v > 1.0f){
		v = 1.0f;
	}else if(v < -1.0f){
		v = -1.0f;
	}
	return(uint16_t)((v + 1.0f) * 500.0f + 1000.0f);
}

void KiwiDrive(float vx, float vy, float omega){
	float Rw = ROTATION_CONST * omega;

	float Mtop = 1.0f *vx + Rw;
	float Mbl = 0.866f*vy -0.5f*vx + Rw;
	float Mbr = -0.866f*vy -0.5f*vx + Rw;

	float maxM = fmaxf(fabsf(Mtop), fmaxf(fabsf(Mbl), fabsf(Mbr)));
	if (maxM > 1.0f) {
	    Mtop /= maxM;
	    Mbl  /= maxM;
	    Mbr  /= maxM;
	}


	//PWM(us)변환 후 TIM1 채널에 출력
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ToPWMus(Mtop)); //PA9 TIM1_CH2
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ToPWMus(Mbl));	 //PA10 TIM1_CH3
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ToPWMus(Mbr));	 //PA11 TIM1_CH4
}


void DebugUART(uint16_t rawX, uint16_t rawY, uint16_t rawZ){
	char buf[128];
	// 초기화된 채널 2, 3, 4에서 PWM 값을 읽어옴
	uint16_t pwm_ch2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
	uint16_t pwm_ch3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
	uint16_t pwm_ch4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);

	int len = snprintf(buf, sizeof(buf),
			"X:%4u, Y:%4u, Z:%4u | M_top(CH2):%4u, M_bl(CH3):%4u, M_br(CH4):%4u\n",
			rawX, rawY, rawZ, pwm_ch2, pwm_ch3, pwm_ch4);

	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
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
