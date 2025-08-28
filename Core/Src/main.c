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
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#include "imrc_ecan.h"
#include "imrc_LD_220MG.h"
#include "imrc_RU_control.h"
#include "imrc_MCU_control.h"
#include "canCtrlConv.h"  //imrc
#include "imrc_PCU_control.h" 
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void connection_monitoring(float); //ESPとの接続確認関数
void unit_check(int wait_time);
void ALL_LED_OFF(void);
void handleMovement(void);
void inertia_injection(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){ // printfを使えるようにする関数
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
  return ch;
}


int reverse = 0; // 0:通常, 1:反転
//---割り込み---
int inertia_flag = 0;



static uint32_t Rx1_unit_code,Rx1_unit_id;
static CAN_RxHeaderTypeDef RxHeader1;
static uint8_t RxData1[8];
static uint32_t id;
static uint8_t data_ESP[8]; //ESPからのデータ
static uint8_t data_PCU[8];
static uint8_t data_MCU1[8];
static uint8_t data_MCU2[8];
static uint8_t data_RU[8];
uint32_t data_type_ESP[2];
int32_t data_type_PCU[2];
uint32_t data_type_MCU1[2];//足回り
uint32_t data_type_MCU2[2];//アーム
uint32_t data_type_RU[2];


uint32_t inertia_start_time = 0;
float connection_time[]={0,0,0,0,0}; //接続確認用　｛WCD,　PCU,　MCU1, MCU2,　RU｝
int Y_ischenge = 0;
uint32_t Rx1_index = 0;
uint32_t Rx1_entry = 0;

 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1){   //CAN割り込み
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK){
    id = (RxHeader1.IDE == CAN_ID_STD)? RxHeader1.StdId : RxHeader1.ExtId;  
    ecan_addrConvertToCodeId(id, &Rx1_unit_code, &Rx1_unit_id, 0);  //unit_code,unit_id 判定
    ecan_headerConvertToIdxEntry(RxData1[0], &Rx1_index, &Rx1_entry);
    
    if(Rx1_unit_code==17 && Rx1_unit_id==1){ //WCD
      connection_time[0] = HAL_GetTick();
      for (int i = 0; i < 8; i++){
        data_ESP[i] = RxData1[i];
      }
      data_type_ESP[0] = Rx1_index;
      data_type_ESP[1] = Rx1_entry; 
      passCANCtrlData(data_ESP);

    }else if(Rx1_unit_code==18 && Rx1_unit_id==1){//PCU
      connection_time[1] = HAL_GetTick();
      for (int i = 1; i < 8; i++){
        data_PCU[i] = RxData1[i];
      }
      data_type_PCU[0] = Rx1_index;
      data_type_PCU[1] = Rx1_entry;

    }else if(Rx1_unit_code==16 && Rx1_unit_id==1){ //MCU1
      connection_time[2] = HAL_GetTick();
      for (int i = 1; i < 8; i++){
        data_MCU1[i] = RxData1[i];
      }
      data_type_MCU1[0] = Rx1_index;
      data_type_MCU1[1] = Rx1_entry;

    }else if(Rx1_unit_code==16 && Rx1_unit_id==2){ //MCU2
      connection_time[3] = HAL_GetTick();
      for (int i = 1; i < 8; i++){
        data_MCU2[i] = RxData1[i];
      }
      data_type_MCU2[0] = Rx1_index;
      data_type_MCU2[1] = Rx1_entry;

    }else if(Rx1_unit_code==19 && Rx1_unit_id==1){ //RU
      connection_time[4] = HAL_GetTick();
      for (int i = 1; i < 8; i++){
        data_RU[i] = RxData1[i];
      }
      data_type_RU[0] = Rx1_index;
      data_type_RU[1] = Rx1_entry;

    }
  }
}


/* USER CODE END Private
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  setbuf(stdout, NULL); 
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  //CAN Setting
  ecan_init(1, 1); //MainBoard
  ecan_setAllPassFilter(&hcan1);
  ecan_setAllPassFilter(&hcan2);
  ecan_start(&hcan1);
  ecan_start(&hcan2);

  //MCU
  MCU_move_Init(&hcan1,1,100); 

  //WCD
  canCtrlConv_Init(100, 10);

  //PCU
  PCU_Init(&hcan1, 1);
  
  //PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  
  unit_check(3000);//接続中のユニットを探す
  
  //Start sign
  HAL_GPIO_TogglePin(BZ_GPIO_Port,BZ_Pin);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(BZ_GPIO_Port,BZ_Pin);
  printf("Start!!\n\r");
  ALL_LED_OFF();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  PCU_voltage_recovery(); //PCUの電圧を復帰

  int Last_reverse_state = 0; 
  int last_LED_state = 0;
  int LED_state;

  int catch_relay_port = 2; //RUのキャッチリレー番号
  int injection_relay_port = 1; //RUの射出リレー番号
  int LED_relay_port = 4; //RUのLEDリレー番号
  while(1){
    connection_monitoring(1700); //各ユニットとの接続確認    
    PCU_survival_signal(1000);  //PCUに生存信号送信

    //---コントローラーのボタン処理---
    allBtnAxiState(); //ボタンの状態を更新
    handleMovement(); //移動（左右スティック）
    
    //--アーム（十字）--
    if(getBtnState(BTN_UP)){ //射出開始
      MCU_injection(&hcan1, 1, 1);
    }else{
      MCU_injection(&hcan1, 1, 0); //射出停止
    }
    
    // if(getBtnState(BTN_DOWN)){
    // }

    int reverse_BTN = getBtnState(BTN_LEFT);//動作反転
    if (reverse_BTN != Last_reverse_state){
      if (reverse_BTN) {
        HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        reverse = !reverse;
      }
      Last_reverse_state = reverse_BTN;
    }

    int LED_BTN = getBtnState(BTN_RIGHT);//LED点灯
    if(LED_BTN != last_LED_state){
      if (LED_BTN) {
        LED_state = !LED_state;
        printf("%d",LED_state);
        RU_control(&hcan1, 1, LED_relay_port, LED_state);//昆虫完成
      }
      last_LED_state = LED_BTN;
    }


    //--アーム（記号）-- 
    if (getBtnState(BTN_A)){//アーム　狙う　(◯)
      MCU_arm_control(&hcan1, 2, arm_aim);
    }else if (getBtnState(BTN_B)){//アーム　キャッチ　(☓)
      MCU_arm_control(&hcan1, 2, arm_catch);
    }else if (getBtnState(BTN_X)) {//アーム　射出　(△)
      MCU_arm_control(&hcan1, 2, arm_injection); 
    }else if (getBtnState(BTN_Y)){//アーム　引きずる（□）
      MCU_arm_control(&hcan1, 2, arm_drag);
    }else{
      MCU_arm_control(&hcan1, 2, arm_null);
    }

    if(data_type_MCU1[0]==3 && data_type_MCU1[1] == 0 && data_MCU1[1] == 1) inertia_flag = 1; //射出命令がMCUから来たら
    if(inertia_flag) inertia_injection(injection_relay_port); //慣性制御

    

    //リレー制御　（トリガー）
    if(getBtnState(BTN_L2)){//アーム開
      RU_control(&hcan1, 1, catch_relay_port, 0);
    }else if(getBtnState(BTN_R2)){//アーム閉
      RU_control(&hcan1, 1, catch_relay_port, 1);//アーム閉
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_A1_Pin|GPIO_A2_Pin|GPIO_A3_Pin|GPIO_B1_Pin
                          |GPIO_B2_Pin|GPIO_B3_Pin|GPIO_C1_Pin|LED3_Pin
                          |LED2_Pin|BZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_D1_Pin|GPIO_D2_Pin|GPIO_D3_Pin|GPIO_D4_Pin
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|CAN_LED2_Pin|CAN_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_A1_Pin GPIO_A2_Pin GPIO_A3_Pin GPIO_B1_Pin
                           GPIO_B2_Pin GPIO_B3_Pin GPIO_C1_Pin LED3_Pin
                           LED2_Pin BZ_Pin */
  GPIO_InitStruct.Pin = GPIO_A1_Pin|GPIO_A2_Pin|GPIO_A3_Pin|GPIO_B1_Pin
                          |GPIO_B2_Pin|GPIO_B3_Pin|GPIO_C1_Pin|LED3_Pin
                          |LED2_Pin|BZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_D1_Pin GPIO_D2_Pin GPIO_D3_Pin GPIO_D4_Pin
                           LED4_Pin */
  GPIO_InitStruct.Pin = GPIO_D1_Pin|GPIO_D2_Pin|GPIO_D3_Pin|GPIO_D4_Pin
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin CAN_LED2_Pin CAN_LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|CAN_LED2_Pin|CAN_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin
                           DIP4_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |DIP4_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP2_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIP2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP1_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIP1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int device_list[5] = {0, 0, 0, 0, 0}; // 接続中のデバイス配列｛WCD,　PCU,　MCU1, MCU2,　RU｝　0:未接続　1:接続中
void  unit_check(int wait_time){
  uint32_t start_time = HAL_GetTick();
  while(HAL_GetTick() - start_time < (wait_time+500)){

    for(int i=0;i<5;i++){
      if(connection_time[i]==0){
        connection_time[i] = HAL_GetTick();
        continue;
      }
      
      float CHECK_TIME = HAL_GetTick() - connection_time[i];
      if(!HAL_GPIO_ReadPin(DIP1_GPIO_Port,DIP1_Pin)) {//全部つながってないといけないモード
        device_list[i] = 1;
        continue;
      }else if(CHECK_TIME > wait_time ){ //つながっていないユニットを探す
        switch (i){
          case 0: //WCD
            printf("WCD is disconnected\n\r");
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
            break;
          case 1: //PCU
            printf("PCU is disconnected\n\r");
            HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
            break;
          case 2: //MCU1
            printf("MCU1 is disconnected\n\r");
            HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
            break;
          case 3: //MCU2
            printf("MCU2 is disconnected\n\r");
            HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
            break;
          case 4: //RU
            printf("RU is disconnected\n\r");
            HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
            break;
        }
        device_list[i] = 0;

      }else{//つながってたら
        device_list[i] = 1;
      }
    }

  }
  
}

void connection_monitoring(float CHECK_INTERVAL) {
  int connection_state[5] = {0, 0, 0, 0, 0}; // 接続確認用の時間配列　｛WCD,　PCU,　MCU1, MCU2,　RU｝ 0:未接続　1:接続中

  for (int i = 0; i < 5; i++) {
    if (connection_time[i] == 0) {
      connection_time[i] = HAL_GetTick();
      continue;
    }

    float CHECK_TIME = HAL_GetTick() - connection_time[i];
    if (CHECK_TIME > CHECK_INTERVAL  && device_list[i] == 1) { //接続が切れたときの処理 
      PCU_voltage_cutoff();

      switch (i) {
        case 0: //WCD
          printf("WCD is disconnected\n\r");
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
          connection_state[0] = 0;
          break;
        case 1: //PCU
          printf("PCU is disconnected\n\r");
          HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
          connection_state[1] = 0;
          break;
        case 2: //MCU1
          printf("MCU1 is disconnected\n\r");
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
          connection_state[2] = 0;  
          break;
        case 3: //MCU2
          printf("MCU2 is disconnected\n\r");
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
          connection_state[3] = 0;  
          break;
        case 4: //RU
          printf("RU is disconnected\n\r");
          HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
          connection_state[4] = 0;
          break;
      }

      // ブザー通知
      HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_SET);
      HAL_Delay(150);
      HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET);
      ALL_LED_OFF();

    }else{
      connection_state[i] = 1;
    }
  }

  int disconect = 0;                      
  for (int i=0 ; i<5 ; i++){
    if(connection_state[i] == 0){
      disconect = 1;
      break;
    }
  }
  //printf("Disconect: %d\n\r", disconect);
  if (disconect) {
    PCU_voltage_cutoff();

  } else {
    PCU_voltage_recovery();

  }

}

void ALL_LED_OFF(void){
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
}

void handleMovement(void){
  int DIR = 0; //移動方向
  int spead = 0; //速度

  //---移動　処理---
  
  if(getAxiState(STK_R_RIGHT)){
    DIR = RIGHT_ROTATE;
    spead = getAxiState(STK_R_RIGHT);
    
  }else if(getAxiState(STK_R_LEFT)){
    DIR = LEFT_ROTATE;
    spead = getAxiState(STK_R_LEFT);

  }else if(getAxiState(STK_L_RIGHT)){
    DIR = RIGHT;
    spead= getAxiState(STK_L_RIGHT);

  }else if (getAxiState(STK_L_UPRIGHT)){
    DIR = FRONT_RIGHT;
    spead = getAxiState(STK_L_UPRIGHT);

  }else if (getAxiState(STK_L_UP)){
    DIR = FRONT;
    spead = getAxiState(STK_L_UP);

  }else if (getAxiState(STK_L_UPLEFT)){
    DIR = FRONT_LEFT;
    spead = getAxiState(STK_L_UPLEFT);

  }else if(getAxiState(STK_L_LEFT)){
    DIR = LEFT;
    spead = getAxiState(STK_L_LEFT);

  }else if(getAxiState(STK_L_DOWNLEFT)){
    DIR = BUCK_LEFT;
    spead = getAxiState(STK_L_DOWNLEFT);

  }else if(getAxiState(STK_L_DOWN)){
    DIR = BUCK;
    spead = getAxiState(STK_L_DOWN);

  }else if(getAxiState(STK_L_DOWNRIGHT)){
    DIR = BUCK_RIGHT;
    spead = getAxiState(STK_L_DOWNRIGHT);

  } else{
    DIR = Stop; // どの方向にも入力がない場合は停止
    spead= 0; // 速度も0に設定

  } 

  //printf("DIR: %d, Speed: %d\n\r", DIR, spead); // デバッグ用出力
  MCU_move(DIR, spead, reverse); //MCUに移動命令
}

void inertia_injection(int relay_NO){
  if (inertia_start_time == 0) {
    inertia_start_time = HAL_GetTick();
    
  }

  if ((HAL_GetTick() - inertia_start_time) < 1200){
    
    RU_control(&hcan1, 1, relay_NO, 1); //慣性制御開始
    printf("Inertia Start %d\n\r",HAL_GetTick() - inertia_start_time);
    
  }else{
    printf("Inertia End\n\r");
    RU_control(&hcan1, 1, relay_NO, 0); //慣性制御終了
    inertia_start_time = 0;
    inertia_flag = 0;
  
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
  MCU_move(Stop, 0, 0);
  __disable_irq();
  while (1)
  {
    printf("Error\n\r");
    HAL_GPIO_WritePin(BZ_GPIO_Port,BZ_Pin,1);
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
