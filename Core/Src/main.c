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
#define ADC_MAX 4020    // 실제 최대값
#define ADC_MIN 0
#define ADC_NEU 2010	//ADC 중간값 4020/2
#define ADC_DEAD_ZONE 200	//데드존 처리 100

#define ROTATION_CONST -0.5f    // 회전 상수

#define RX_TIMEOUT_MS 100	//안정장치-100ms동안 조종기 신호가 없으면 통신이 끊겼다고 판단하고 모터를 정지시킴


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // 수신 파이프 주소를 송신부와 동일하게 설정

//interrupt flag
volatile uint8_t nrf_irq_flag = 0;
volatile uint8_t watchdog_flag = 0;

//system state
static uint16_t last_rx_ms = 0;
static uint16_t pwm_active = 0;
static uint32_t no_signal_count = 0;	//시그널이 없을때 UART디버그용

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//함수선언
void nrf24_receiver_setup(void);
void nrf24_irq_service(void);
void system_watchdog_service(void); // ★ Watchdog service function
float NormalizeADC(int16_t delta);
uint16_t ToPWMus(float value);
void KiwiDrive(float vx, float vy, float omega);
void DebugUART(uint16_t rawX, uint16_t rawY, uint16_t rawZ);
void PWM_StartNeutral(void);
void PWM_StopAll(void);
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
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // nRF24 초기화 (수신기)
  nrf24_init();
  nrf24_receiver_setup();
  HAL_TIM_Base_Start_IT(&htim3);	//TIM3 시작

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //---데이터 수신 이벤트 처리---
	  if(nrf_irq_flag){
		  nrf_irq_flag = 0;	//flag 내리기
		  nrf24_irq_service();	//데이터 처리 함수 호출, 모터 제어
	  }
	  //---TIM3기반 와치독 이벤트 처리---
	  if(watchdog_flag){
		  watchdog_flag = 0;	//확인 후 flag 내림
		  system_watchdog_service();	//와치독 함수 호출
	  }

	  //---저전력 모드 진입(WFI)---
	  if(!nrf_irq_flag && !watchdog_flag){	//두개의 flag가 내려가 있으면 처리할 이벤트가 없으므로 WFI
		  __WFI();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_Pin_GPIO_Port, CSN_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_Pin_GPIO_Port, CE_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CSN_Pin_Pin */
  GPIO_InitStruct.Pin = CSN_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSN_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin_Pin */
  GPIO_InitStruct.Pin = CE_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : E_Stop_Pin */
  GPIO_InitStruct.Pin = E_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_Stop_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//nRF24L01 모듈이 데이터를 수신하면 PC IRQ핀에 하강엣지 트리거 발생, 위 콜백함수 호출
	if(GPIO_Pin == GPIO_PIN_8){	//이 인터럽트가 PC8핀에서 발생했으면
		nrf_irq_flag = 1;	//Main 루프에 데이터 도착 플래그 올림
	}else if(GPIO_Pin == GPIO_PIN_6){
		//비상 정지버튼 E-STOP에서 인터럽트가 발생했다면
		PWM_StopAll();	//모든 pwm중지
	}
}

void nrf24_irq_service(void){
	nrf24_stop_listen();
	uint8_t st = nrf24_r_reg(STATUS, 1);

	if(st & (1<<6)){
		uint8_t buf[6];
		nrf24_receive(buf,6);

		// 6바이트 2진 언패킹(unpacking)
		uint16_t rawX = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
		uint16_t rawY = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
		uint16_t rawZ = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

		if(!pwm_active){
			PWM_StartNeutral();
		}

		//읽어온 값을 실제 모터 제어값으로 정규화
		float vx = NormalizeADC((int16_t)rawX); // 부호 있어야함
		float vy = NormalizeADC((int16_t)rawY);
		float omega = NormalizeADC((int16_t)rawZ);

		//변환된 값으로 키위 드라이브 알고리즘을 실행, 모터 구동
		KiwiDrive(vx, vy, omega);

		//TIM3 워치독을 위해 마지막으로 데이터를 수신한 시간을 현재시간으로 갱신
		last_rx_ms = HAL_GetTick();

		//nrf24의 RX_DR상태 비터를 0으로 claer, 다음 인터럽트 받을 준비
		nrf24_clear_rx_dr();
	}

	nrf24_listen();	//데이터 수신 대기모드

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	//50ms마다 발생하는 인터럽트(TIM3)
	if(htim->Instance == TIM3){	//이 인터럽트가 TIM3에서 발생했으면
		watchdog_flag = 1;	//Main 루프에 점검할 시간이라고 플래그 올림
	}
}

void system_watchdog_service(void){	//워치독 서비스
	if(pwm_active){
		//모터가 동작 중일때만 감시 수행
		if((HAL_GetTick() - last_rx_ms) > RX_TIMEOUT_MS){	//현재시간과 마지막 데이터 수신 시간의 차이가 타임아웃(100ms)를 초과했다면
			PWM_StopAll();	//모든 모터 정지
		}
	}
}

void PWM_StartNeutral(void){
	//초기 PWM 중립 세팅
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);

    //PWM 시작
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	pwm_active = 1;	//시스템 상태를 pwm 활성화로 변경하는 플래그
}

//모든 채널의 PWM 완전 정지
void PWM_StopAll(void){
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    pwm_active = 0;
}

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

    uint8_t cfg = nrf24_r_reg(CONFIG, 1);
    // TX_DS와 MAX_RT 인터럽트는 비활성화(Mask)하고, RX_DR만 남겨둔다.
    cfg |= (1 << 5) | (1 << 4); // Bit 5와 4를 1로 만듦(비활성화)
    cfg &= ~(1 << 6);           // Bit 6은 0으로 만듦 (RX_DR 활성화)
    nrf24_w_reg(CONFIG, &cfg, 1);

    nrf24_open_rx_pipe(0, rx_address);              //파이프 0에 수신주소 설정
    nrf24_listen();                                 //CE=high => 실제 수신 대기모드 진입
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);	//인터럽트 플래그 초기화

}

/*
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
*/

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
