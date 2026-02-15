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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "imrc_ecan.h"
#include "imrc_LD_220MG.h"
#include "imrc_RU_control.h"
#include "imrc_MCU_control.h"
#include "canCtrlConv.h"  //imrc
#include "imrc_PCU_control.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "imrc_connect_monitor.h"
#include "imrc_main.h"


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
#define ENABLED_PRINTF 1

#if ENABLED_PRINTF
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) ((void)0) // 何もしない
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){ // printfを使えるようにする関数
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
	return ch;
}

static uint32_t Rx1_unit_code,Rx1_unit_id;
static CAN_RxHeaderTypeDef RxHeader1;
static uint8_t RxData1[8];
static uint32_t id1;
uint32_t Rx1_index = 0;
uint32_t Rx1_entry = 0;
static uint32_t Rx2_unit_code,Rx2_unit_id;
static CAN_RxHeaderTypeDef RxHeader2;
static uint8_t RxData2[8];
static uint32_t id2;
uint32_t Rx2_index = 0;
uint32_t Rx2_entry = 0;




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){   //CAN割り込み
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK){
		id1 = (RxHeader1.IDE == CAN_ID_STD)? RxHeader1.StdId : RxHeader1.ExtId;  
		ecan_addrConvertToCodeId(id1, &Rx1_unit_code, &Rx1_unit_id, 0);  //unit_code,unit_id 判定
		ecan_headerConvertToIdxEntry(RxData1[0], &Rx1_index, &Rx1_entry);

    if(Rx1_unit_code==18 && Rx1_unit_id==1){ //PCU
      conn_rx(1 ,HAL_GetTick());

		}else if(Rx1_unit_code==16 && Rx1_unit_id==1){ //MCU1
      conn_rx(2 ,HAL_GetTick());

		}else if(Rx1_unit_code==16 && Rx1_unit_id==2){ //MCU2
      conn_rx(3 ,HAL_GetTick());

		}else if(Rx1_unit_code==20 && Rx1_unit_id==1){ //LCU
      conn_rx(4 ,HAL_GetTick());

	  }

  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK){
		id2 = (RxHeader2.IDE == CAN_ID_STD)? RxHeader2.StdId : RxHeader2.ExtId;  
		ecan_addrConvertToCodeId(id2, &Rx2_unit_code, &Rx2_unit_id, 0);  //unit_code,unit_id 判定
		ecan_headerConvertToIdxEntry(RxData2[0], &Rx2_index, &Rx2_entry);

		}
	}
	

//生存信号
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//タイマー割り込み　生存信号
	if (htim->Instance == TIM5){
        ecan_sendEmptyPacketMtoU(&hcan2, 18, 1, 0, 5);
    }
}

//UART割り込み
#define RX_BUFFER_SIZE 64
uint8_t UART_PC_RAWdata;
uint8_t UART_PC_data[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t PC_UART_complete = 0;
uint8_t PC_uart_started = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART1) {//PC
    if (!PC_uart_started) {
      if (UART_PC_RAWdata == '#') {//#だったら開始
        PC_uart_started = 1;
        rx_index = 0;
        //rx_buffer[rx_index++] = rx_data; // # も格納
      }

    } else {
      if (UART_PC_RAWdata == '\r' || UART_PC_RAWdata == '\n') {
        UART_PC_data[rx_index] = '\0';
        PC_UART_complete = 1;
        PC_uart_started = 0; // 次回まで待機
        rx_index = 0;
      } else {
        if (rx_index < RX_BUFFER_SIZE - 1) {
          UART_PC_data[rx_index++] = UART_PC_RAWdata;
        } else {
          // バッファオーバーフロー時リセット
          rx_index = 0;
          PC_uart_started = 0;
          PC_UART_complete = 0;
        }
      }
    }

    HAL_UART_Receive_IT(&huart1, &UART_PC_RAWdata, 1);
  }
}


/* USER CODE END Private */
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//CAN Setting
	ecan_init(1, 1); //MainBoard

	ecan_setAllPassFilter_advance(&hcan1, CAN_FILTER_FIFO0, 0, 14);
	ecan_start_advance(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	ecan_setAllPassFilter_advance(&hcan2, CAN_FILTER_FIFO1, 14, 14);
  ecan_start_advance(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	
	//PWM
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //未使用
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //未使用
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //未使用
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim5); // TIM5 割り込みスタート

	HAL_UART_Receive_IT(&huart1, &UART_PC_RAWdata, 1);

  unit_t units[] = {
    { "PCU"   , 1, 0, 0, 1000 },
    { "MCU1"  , 2, 0, 0, 1000 },
    { "MCU2"  , 3, 0, 0, 1000 },
    { "LCU"   , 4, 0, 0, 1000 },
  };
  conn_init(units, 4, 1);
  HAL_Delay(1000); //ユニット検出
  conn_update(HAL_GetTick());
  conn_state();

  HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, 1);
  HAL_Delay(100); //ユニット検出
  HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, 0);

	printf("Start!!\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    if (PC_UART_complete){

      pc_uart pc_data;
      parse_csv_to_struct(UART_PC_data, &pc_data);

      //debug
      /*
      DEBUG_PRINTF("moduleName: %s, topiNacme: %s, dataLen: %d     ",pc_data.module_name,pc_data.topic_name,pc_data.data_len);
      for(int i=0;i<pc_data.data_len;i++){
        DEBUG_PRINTF("[%f]",pc_data.data[i]);
      }
      DEBUG_PRINTF("\n\r");
      */

      if(*pc_data.module_name == 'M'){
        if(strcmp(pc_data.topic_name, "cmd_vel") == 0){
          uint8_t sent_data[] = {clampTo255(pc_data.data[0]),clampTo255(pc_data.data[1]),clampTo255(pc_data.data[2] /1.7)};
          //DEBUG_PRINTF("%d",clampTo255(pc_data.data[0]));
          ecan_sendPacketMtoU(&hcan1, 16, 1, 3, 0, 3, sent_data);
        }

      }else if(*pc_data.module_name == 'L'){
        if(strcmp(pc_data.topic_name, "led_cmd") == 0){
          uint8_t sent_data[] = {pc_data.data[0], pc_data.data[1], pc_data.data[2], pc_data.data[3], pc_data.data[4], pc_data.data[5]};
          //sent_data = {LED_number, mode, r, g, b, period}
          ecan_sendPacketMtoU(&hcan1, 20, 1, 3, 0, pc_data.data_len, sent_data);
          
        }

      }


      PC_UART_complete = 0;
    }

    conn_update(HAL_GetTick());
    conn_chack();

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
  RCC_OscInitStruct.PLL.PLLN = 120;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/*その他*/
void ALL_LED_OFF(void){//全LED消灯
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAN_LED1_GPIO_Port,CAN_LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAN_LED2_GPIO_Port,CAN_LED2_Pin,GPIO_PIN_RESET);
}


void Buzzer_toggle(uint32_t interval_time){
    static uint32_t last_tick = 0;
    static uint8_t buzzer_state = 0; // 0: OFF, 1: ON

    uint32_t now = HAL_GetTick();

    if (now - last_tick >= interval_time) {
        last_tick = now;
        buzzer_state = !buzzer_state;

        if (buzzer_state) {
            HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_SET); // ON
        } else {
            HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET); // OFF
        }
    }
}



//ライブラリに入れたいなーーー
#define MAX_BUTTONS 16 
static uint32_t lastPressTime[MAX_BUTTONS] = {0};
bool getBtnMultiState(const int *buttons, uint8_t numButtons, uint32_t threshold){
	uint32_t now = HAL_GetTick();
	uint32_t minTime = 0xFFFFFFFF;
	uint32_t maxTime = 0;
	uint8_t pressedCount = 0;

	// すべての対象ボタンについて処理
	for (uint8_t i = 0; i < numButtons; i++) {
		uint8_t btn = buttons[i];

		if (getBtnState(btn)) {
			// ボタンが押されていたら時刻を更新
			if (lastPressTime[btn] == 0) {
				lastPressTime[btn] = now;
			}

			// 最小と最大の押下時刻を記録
			if (lastPressTime[btn] < minTime) minTime = lastPressTime[btn];
			if (lastPressTime[btn] > maxTime) maxTime = lastPressTime[btn];

			pressedCount++;
		} else {
			// 離されていたら時刻リセット
			lastPressTime[btn] = 0;
		}
	}

	// 全部押されていて、かつ押下時刻の差が threshold 以内なら同時押し
	if (pressedCount == numButtons && (maxTime - minTime) <= threshold) {
		return 1;
	}

	return 0;
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
	MCU_move(Stop, 0, 0,0);
	PCU_voltage_cutoff();
	__disable_irq();
	while (1)
	{
		printf("Error\n\r");
		HAL_GPIO_WritePin(BZ_GPIO_Port,BZ_Pin,1);
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
		 ex: DEBUG_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
