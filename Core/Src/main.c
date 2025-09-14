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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define ENABLED_PRINTF 1

#if ENABLED_PRINTF
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) ((void)0) // 何もしない
#endif


//プログラム内　グローバル変数
int is_start_move_to_aim_position = 0;
int is_start_calibration = 0;
int is_start_injection_release = 0;
int is_start_injection_set = 0;
int is_start_drag = 0;
int is_start_beetle_set = 0;
int is_start_reload_from_drag = 0;
int is_start_move_to_catch = 0;
int is_start_all_injection = 0;
int is_start_Black_injection = 0;
int is_start_White_injection = 0;
int is_start_LED = 0;

int charge_FLAG = 0;

int last_arm_command = 0;
int  running_arm_command = 0;

int reverse = 0; // 0:通常, 1:反転
int inertia_flag = 0;
uint32_t inertia_start_time = 0;
int catch_state = 0;
int null;
int Black_charge_state = 0; //射出チャージ時:1 非チャージ時:0
int White_charge_state = 0; //射出チャージ時:1 非チャージ時:0
int arm_position = 10;
TIM_HandleTypeDef *Black_lock_Servo_HT;
uint32_t Black_lock_Servo_CN;
TIM_HandleTypeDef *White_lock_Servo_HT;
uint32_t White_lock_Servo_CN;

typedef struct {//長押し用構造体
    uint32_t press_start_time;
    int is_pressed;
    int BTN_state;
} ButtonHandle;

ButtonHandle Black_injection_release_BTN = {0, 0, 0};
ButtonHandle White_injection_release_BTN = {0, 0, 0};
ButtonHandle LED_BTN = {0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void connection_monitoring(float); //ESPとの接続確認関数
void unit_check(int wait_time);
void ALL_LED_OFF(void);
void handleMovement(void);
void inertia_injection(int);
void injection_Init(int, TIM_HandleTypeDef *, uint32_t , TIM_HandleTypeDef *, uint32_t );
bool injection_set(void);
int injection_release(int, int);
bool getBtnMultiState(const int *, uint8_t , uint32_t);
bool injection_charge(void);
int arm_drag_set(int);
void catch_open(int);
void catch_close(int);
bool injection_reload_from_drag(int);
int MCU_arm_control(int, int);
int send_calibration_signal(void);
void injection_FLAG_reset(void);
void arm_FLAG_reset(void);
bool getBtnHoldState(ButtonHandle *, uint32_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){ // printfを使えるようにする関数
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
	return ch;
}

//割り込み
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

static uint8_t data_ESP[8]; //ESPからのデータ
static uint8_t data_PCU[8];
static uint8_t data_MCU1[8];
static uint8_t data_MCU2[8];
static uint8_t data_RU[8];
uint8_t data_type_ESP[2];
uint8_t data_type_PCU[2];
uint8_t data_type_MCU1[2];//足回り [index, entry]
uint8_t data_type_MCU2[2];//アーム [index, entry]
uint8_t data_type_RU[2];
float connection_time[]={0,0,0,0,0}; //接続確認用　｛WCD,　PCU,　MCU1, MCU2,　RU｝

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){   //CAN割り込み
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK){
		id1 = (RxHeader1.IDE == CAN_ID_STD)? RxHeader1.StdId : RxHeader1.ExtId;  
		ecan_addrConvertToCodeId(id1, &Rx1_unit_code, &Rx1_unit_id, 0);  //unit_code,unit_id 判定
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

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader2, RxData2) == HAL_OK){
		id2 = (RxHeader2.IDE == CAN_ID_STD)? RxHeader2.StdId : RxHeader2.ExtId;  
		ecan_addrConvertToCodeId(id2, &Rx2_unit_code, &Rx2_unit_id, 0);  //unit_code,unit_id 判定
		ecan_headerConvertToIdxEntry(RxData2[0], &Rx2_index, &Rx2_entry);
		
		if(Rx2_unit_code==18 && Rx2_unit_id==1){//PCU
			connection_time[1] = HAL_GetTick();
			for (int i = 1; i < 8; i++){
				data_PCU[i] = RxData2[i];
			}
			data_type_PCU[0] = Rx2_index;
			data_type_PCU[1] = Rx2_entry;

		}
	}
	

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//タイマー割り込み　生存信号
	if (htim->Instance == TIM5){
        ecan_sendEmptyPacketMtoU(&hcan2, 18, 1, 0, 5);
    }
}


//ピン命名
#define GPIO_D1_TIM_CN TIM_CHANNEL_4
#define GPIO_D3_TIM_CN TIM_CHANNEL_1
#define GPIO_D4_TIM_CN TIM_CHANNEL_1
TIM_HandleTypeDef *GPIO_D1_TIM_HT = &htim2; 
TIM_HandleTypeDef *GPIO_D3_TIM_HT = &htim2; 
TIM_HandleTypeDef *GPIO_D4_TIM_HT = &htim3; 

// #define GPIO_C2_TIM_CN TIM_CHANNEL_1 //未使用
// #define GPIO_C3_TIM_CN TIM_CHANNEL_2 //未使用
// #define GPIO_C4_TIM_CN TIM_CHANNEL_3 //未使用
// TIM_HandleTypeDef *GPIO_C2_TIM_HT = &htim2; //未使用
// TIM_HandleTypeDef *GPIO_C3_TIM_HT = &htim2; //未使用
// TIM_HandleTypeDef *GPIO_C4_TIM_HT = &htim2; //未使用

//入力ピン　定義
int charge_valve = 1;
int catch_relay_port = 3; //RUのキャッチリレー番号
int LED_relay_port = 4; //RUのLEDリレー番号

int SW3_last_state = 0;

int Black_initial_position = 40;
int White_initial_position = 140;
int Black_lock_position = 10;
int White_lock_position = 170;



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
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	
	//CAN Setting
	ecan_init(1, 1); //MainBoard

	ecan_setAllPassFilter_advance(&hcan1, CAN_FILTER_FIFO0, 0, 14);
	ecan_start_advance(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	ecan_setAllPassFilter_advance(&hcan2, CAN_FILTER_FIFO1, 14, 14);
  	ecan_start_advance(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	//MCU
	MCU_move_Init(&hcan1, 1, 100); 
	MCU_arm_Init(&hcan1, 2, &arm_position, data_MCU2, data_type_MCU2);

	//WCD
	canCtrlConv_Init(100, 10);

	//PCU
	PCU_Init(&hcan2, 1);
	
	//PWM
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //未使用
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //未使用
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //未使用
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim5); // TIM5 割り込みスタート


	//射出
	injection_Init(charge_valve, GPIO_D1_TIM_HT, GPIO_D1_TIM_CN, GPIO_D3_TIM_HT, GPIO_D3_TIM_CN);
	LD_220MG_SetAngle(Black_lock_Servo_HT, Black_lock_Servo_CN, Black_initial_position); //ロック外し 初期位置
	LD_220MG_SetAngle(White_lock_Servo_HT, White_lock_Servo_CN, White_initial_position); //ロック外し 初期位置
	 
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

	//ローカル変数
	int Last_reverse_state = 0; 
	int Last_caliblation_state= 0;
	int last_LED_state = 0;
	int LED_state = 0;

	while(1){
		connection_monitoring(1700); //各ユニットとの接続確認

		//ボタン指定
		int beetle_set_BTN = !HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin); //カブトムシ装填
		int send_calibration_BTN[] = {BTN_R1 , BTN_LEFT}; //キャリブレーション信号送信

		int set_injection_BTN = getBtnState(BTN_X);  //射出装填 △
		int arm_move_to_aim_position_BTN = getBtnState(BTN_Y);  //アーム狙う位置　□
		int arm_move_to_catch_position_BTN = getBtnState(BTN_A); //アームキャッチ位置　〇
		int arm_move_to_drag_BTN = getBtnState(BTN_B); //アーム引きずり位置　×

		//int reload_from_drag_BTN = getBtnState(BTN_UP); //自動装填
		int reload_from_drag_BTN = 0; //自動装填   
		//int charge_BTN = getBtnState(BTN_UP);
		int charge_BTN = 0;

		int move_reverse_BTN = getBtnState(BTN_DOWN);  //動作反転
		LED_BTN.BTN_state = getBtnState(BTN_LEFT); //昆虫図鑑完成
		//int LED_BTN = getBtnState(BTN_LEFT); //昆虫完成
		
		int ALL_injection_buttons[]={BTN_RIGHT , BTN_L1}; //全射出
		White_injection_release_BTN.BTN_state = getBtnState(BTN_L1); //白射出（長押し）
		Black_injection_release_BTN.BTN_state = getBtnState(BTN_RIGHT); //黒射出（長押し）

		int catch_open_BTN = getBtnState(BTN_L2); //つかむ機構　開
		int catch_close_BTN = getBtnState(BTN_R2); //つかむ機構　閉
		//R1は速度を30に設定（handleMovement内）

		int send_caliblation_BTN = getBtnMultiState(send_calibration_BTN, 2, 30);
		int ALL_injection_BTN = getBtnMultiState(ALL_injection_buttons, 2, 30);

		//DEBUG_PRINTF("curent position: %d\n\r",arm_position);
		//DEBUG_PRINTF("change state  release1 : %d  release2 : %d \n\r", Black_charge_state, White_charge_state);

		//---コントローラーのボタン処理---
		allBtnAxiState(); //ボタンの状態を更新
		handleMovement(); //移動（左右スティック） R1で速度を遅くする

		/*-----------フラグ-----------*/
		//アーム　フラグ建築
		if(send_caliblation_BTN != Last_caliblation_state){ 
			if(send_caliblation_BTN ){
				arm_FLAG_reset();
				is_start_calibration = 1;
			}

			Last_caliblation_state = send_caliblation_BTN;
		}else if(reload_from_drag_BTN){
			arm_FLAG_reset();
			is_start_reload_from_drag = 1; //引きずる機構から再装填 新
		}else if(set_injection_BTN){
			arm_FLAG_reset();
			is_start_injection_set = 1;   //射出装填　新
		}else if(arm_move_to_catch_position_BTN){
			arm_FLAG_reset();
			is_start_move_to_catch = 1;
		}else if(arm_move_to_drag_BTN){
			arm_FLAG_reset();
			is_start_drag= 1; //引きずる機構に移動(自動)　新
		}else if(arm_move_to_aim_position_BTN){
			arm_FLAG_reset();
			is_start_move_to_aim_position = 1;
		}else if(getBtnHoldState(&LED_BTN,40) && getBtnState(BTN_LEFT) != last_LED_state){
			if(getBtnState(BTN_LEFT)){
				DEBUG_PRINTF("LED\n\r");
				is_start_LED = 1;
				
			}
			last_LED_state = getBtnState(BTN_LEFT);
			
		}

	
		//射出　フラグ
		if(ALL_injection_BTN && charge_FLAG == 0){
			injection_FLAG_reset();
			

			is_start_all_injection = 1;
		}else if(getBtnHoldState(&White_injection_release_BTN, 40) && charge_FLAG == 0){
			injection_FLAG_reset();


			is_start_White_injection = 1;
		}else if(getBtnHoldState(&Black_injection_release_BTN, 40) && charge_FLAG == 0){
			injection_FLAG_reset();
			

			is_start_Black_injection = 1;
		}else{
			injection_FLAG_reset();

		}

		if(charge_BTN){
			charge_FLAG = 1;
		}




		/*-----------処理-----------*/
		//アーム　処理
		if(is_start_calibration){ //キャリブレーション送信
			if(send_calibration_signal())	is_start_calibration = 0;

		}else if(is_start_move_to_aim_position){  //箱狙う位置　ok
			if(MCU_arm_control(aim_position, 2500))	is_start_move_to_aim_position = 0;
			
		}else if(is_start_injection_set){ //射出装填
			if(injection_set())	is_start_injection_set = 0;
			//if(is_start_injection_set) injection_charge(1, 1); //(チャージのみ)

		}else if(is_start_move_to_catch){ //アームキャッチポジ
			if(MCU_arm_control(catch_position, 2500)) is_start_move_to_catch = 0; //とる位置まで移動　ok

		}else if(is_start_drag){ //アーム箱位置
			if(arm_drag_set(1000))	is_start_drag = 0;
		}
		// else if(is_start_reload_from_drag){//引きずりから自動装てん
		// 	if(injection_reload_from_drag(1000) ) is_start_reload_from_drag = 0;
		// 	DEBUG_PRINTF("a\n\r");
		// }
		
		//射出,チャージ　処理
		if(charge_FLAG){
			DEBUG_PRINTF("charge\n\r");
			if(injection_charge()) charge_FLAG = 0; 
		}else if(!(arm_position == injection_position && catch_state == 1)){
			if(is_start_all_injection){ //二つのボタン、両射出
				if(injection_release(1, 1)) is_start_all_injection = 0;
				DEBUG_PRINTF("ALL\n\r");
			}else if(is_start_White_injection){//白射出
				if(injection_release(0, 1)) is_start_White_injection = 0;
				DEBUG_PRINTF("White\n\r");
			}else if(is_start_Black_injection){//黒射出
				if(injection_release(1, 0)) is_start_Black_injection = 0;
				DEBUG_PRINTF("Black\n\r");
			}

		}
		
		

		

		//つかむ機構
		if(catch_open_BTN){//つかむアーム開 ok
			catch_open(1);
		}else if(catch_close_BTN){//つかむアーム閉 ok
			catch_close(1);
		}

		//その他
		if (move_reverse_BTN != Last_reverse_state){ //動作反転
			if (move_reverse_BTN) {
				reverse = !reverse;
			}
			Last_reverse_state = move_reverse_BTN;
			
		}
		if(reverse == 1) RU_Toggle_relay(&hcan1, 1, LED_relay_port,1000,300);

		

		if(beetle_set_BTN == 1) is_start_beetle_set = 1; //カブトムシ装填
		if(is_start_beetle_set){
			if(injection_charge() ) is_start_beetle_set = 0;
		} 


		if(Black_charge_state == 1 || White_charge_state == 1){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
		}else if(Black_charge_state == 0 && White_charge_state == 0){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		}



		if(!HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)){//射出
			injection_release(1, 1);
		}

		if(!HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) !=  SW3_last_state){//キャリブレーション
			if(!HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)){
				is_start_calibration = 1;
			}
			
			SW3_last_state = !HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);

		}

		//sw4 チャージ

		if(is_start_LED == 1){ //昆虫図鑑完成
			LED_state = !LED_state;
			DEBUG_PRINTF("LED State:%d\n\r",LED_state);
			RU_control(&hcan1, 1, LED_relay_port, LED_state);//昆虫完成
			is_start_LED = 0;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 59;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 59999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 200;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_C2_Pin|GPIO_D2_Pin|LED4_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : GPIO_C2_Pin GPIO_D2_Pin LED4_Pin */
  GPIO_InitStruct.Pin = GPIO_C2_Pin|GPIO_D2_Pin|LED4_Pin;
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

  /*Configure GPIO pins : SW4_Pin SW3_Pin SW2_Pin SW1_Pin
                           DIP4_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = SW4_Pin|SW3_Pin|SW2_Pin|SW1_Pin
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

/*その他*/
void ALL_LED_OFF(void){//全LED消灯
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAN_LED1_GPIO_Port,CAN_LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAN_LED2_GPIO_Port,CAN_LED2_Pin,GPIO_PIN_RESET);
}

void arm_FLAG_reset(void){
	is_start_calibration = 0;
	is_start_reload_from_drag = 0;
	is_start_injection_set = 0;
	is_start_move_to_catch = 0;
	is_start_drag = 0;
	is_start_move_to_aim_position = 0;
}

void injection_FLAG_reset(void){
	is_start_White_injection = 0;
	is_start_Black_injection = 0;
	is_start_all_injection = 0;

}

void BZ_ON(int On_Time){
	static uint32_t BZ_Start_time = 0;
	if(BZ_Start_time == 0) BZ_Start_time = HAL_GetTick();
	if(HAL_GetTick()-BZ_Start_time <= On_Time ){
		HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, 1);
	}else{
		HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, 0);
		BZ_Start_time = 0;
	}
}


/*接続確認*/
int device_list[5] = {0, 0, 0, 0, 0}; // 接続中のデバイス配列｛WCD,　PCU,　MCU1, MCU2,　RU｝　0:未接続　1:接続中
char *device_name[5] = {"WCD", "PCU", "MCU1", "MCU2", "RU"};
void  unit_check(int wait_time){//接続中のユニットを探す
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
				device_list[i] = 0;
				switch (i){
					case 0: //WCD
						//DEBUG_PRINTF("WCD is disconnected\n\r");
						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
						break;
					case 1: //PCU
						//DEBUG_PRINTF("PCU is disconnected\n\r");
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
						break;
					case 2: //MCU1
						//DEBUG_PRINTF("MCU1 is disconnected\n\r");
						HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
						break;
					case 3: //MCU2
						//DEBUG_PRINTF("MCU2 is disconnected\n\r");
						HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
						break;
					case 4: //RU
						//DEBUG_PRINTF("RU is disconnected\n\r");
						HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
						break;
				}
			}else{//つながってたら
				device_list[i] = 1;
			}
		}

	}
	for(int i=0;i<5;i++){
		if(i==0) DEBUG_PRINTF("---Device state---\n\r");
		if(device_list[i] == 0) DEBUG_PRINTF("%s\tcan't find\n\r",device_name[i]);
		if(i==4) DEBUG_PRINTF("------------------\n\r");
	}
	
}

void connection_monitoring(float CHECK_INTERVAL) {//各ユニットとの接続確認
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
					DEBUG_PRINTF("WCD is disconnected\n\r");
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
					connection_state[0] = 0;
					break;
				case 1: //PCU
					DEBUG_PRINTF("PCU is disconnected\n\r");
					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
					connection_state[1] = 0;
					break;
				case 2: //MCU1
					DEBUG_PRINTF("MCU1 is disconnected\n\r");
					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); //led3
					connection_state[2] = 0;  
					break;
				case 3: //MCU2
					DEBUG_PRINTF("MCU2 is disconnected\n\r");
					HAL_GPIO_WritePin(CAN_LED1_GPIO_Port, CAN_LED1_Pin, GPIO_PIN_SET); //led3
					connection_state[3] = 0;  
					break;
				case 4: //RU
					DEBUG_PRINTF("RU is disconnected\n\r");
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
					connection_state[4] = 0;
					break;
			}

			// ブザー通知
			HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_SET);
			HAL_Delay(150);
			HAL_GPIO_WritePin(BZ_GPIO_Port, BZ_Pin, GPIO_PIN_RESET);
			//ALL_LED_OFF();

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
	//DEBUG_PRINTF("Disconect: %d\n\r", disconect);
	if (disconect) {
		PCU_voltage_cutoff();
		injection_release(1,1);

	} else {
		PCU_voltage_recovery();
		// LD_220MG_SetAngle(Black_lock_Servo_HT, Black_lock_Servo_CN, Black_initial_position); //ロック外し 初期位置
		// LD_220MG_SetAngle(White_lock_Servo_HT, White_lock_Servo_CN, White_initial_position); //ロック外し 初期位置


	}

}

/*移動*/
void handleMovement(void){//移動　スティック
	int DIR = 0; //移動方向
	int spead = 0; //速度
	static int slow_move = 0;

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

	if(getBtnState(BTN_R1)){
		slow_move=1;
	} else{
		slow_move=0;
	} 
	//DEBUG_PRINTF("DIR: %d, Speed: %d\n\r", DIR, spead); // デバッグ用出力
	MCU_move(DIR, spead, reverse,slow_move); //MCUに移動命令
}


/*射出関係*/
void inertia_injection(int relay_NO){//慣性射出
	if (inertia_start_time == 0) {
		inertia_start_time = HAL_GetTick();
		
	}

	if ((HAL_GetTick() - inertia_start_time) < 1200){
		
		RU_control(&hcan1, 1, relay_NO, 1); //慣性制御開始
		//DEBUG_PRINTF("Inertia Start %d\n\r",HAL_GetTick() - inertia_start_time);
		
	}else{
		DEBUG_PRINTF("Inertia End\n\r");
		RU_control(&hcan1, 1, relay_NO, 0); //慣性制御終了
		inertia_start_time = 0;
		inertia_flag = 0;
	
	}
}

void injection_Init(int __charge_valve, TIM_HandleTypeDef *__Black_lock_Servo_HT, uint32_t __Balck_lock_Servo_CN, TIM_HandleTypeDef *__White_lock_Servo_HT, uint32_t __White_lock_Servo_CN){//1:正面右　2:正面左
	charge_valve = __charge_valve;
	Black_lock_Servo_HT = __Black_lock_Servo_HT;
	Black_lock_Servo_CN = __Balck_lock_Servo_CN;
	White_lock_Servo_HT = __White_lock_Servo_HT;
	White_lock_Servo_CN = __White_lock_Servo_CN;
}

bool injection_charge(void){//射出チャージ　自動
	static uint32_t injection_start_time = 0;
	static int Black_charge_doProsess = 0;
	static int White_charge_doProsess = 0;

	if(Black_charge_state == 0 || White_charge_state == 0){//どちらかがチャージされていないとき
		if(injection_start_time == 0){//1
			DEBUG_PRINTF("injection_set\n\r");
			injection_start_time  = HAL_GetTick();
			RU_control(&hcan1, 1, charge_valve, 1); 
		}
 
		//--射出機構　チャージ--
		uint32_t check_time = HAL_GetTick() - injection_start_time;
		if(check_time >= 2000){//3  2500
			RU_control(&hcan1, 1, charge_valve, 0);			
			Black_charge_state = 1; //チャージ状況保存
			White_charge_state = 1; 
			injection_start_time = 0;
			return true; //装填プログラム実装でき次第削除

		}else if(check_time >=1200){//2  1500
			LD_220MG_SetAngle(Black_lock_Servo_HT,  Black_lock_Servo_CN, Black_lock_position); //射出ロック
			LD_220MG_SetAngle(White_lock_Servo_HT,  White_lock_Servo_CN, White_lock_position); 

			
		}
	}else{
		injection_start_time = 0;
		return true;
	}

	return false;

}

int injection_release(int Black_release_doProsess, int White_release_doProsess){//射出!!　
	static uint32_t injection_release_timecount = 0;

	if(Black_charge_state == 0 && White_charge_state == 0){ //どちらもチャージされていない
		return 1;
	}else{
		if(Black_release_doProsess){
			LD_220MG_SetAngle(Black_lock_Servo_HT,  Black_lock_Servo_CN, Black_initial_position);  //ロック解除
			Black_charge_state = !Black_release_doProsess; //チャージ状況保存
		} 
		if(White_release_doProsess){
			LD_220MG_SetAngle(White_lock_Servo_HT,  White_lock_Servo_CN, White_initial_position);  //ロック解除
			White_charge_state = !White_release_doProsess; //チャージ状況保存
		}
		DEBUG_PRINTF("change state  release1 : %d  release2 : %d \n\r", Black_charge_state, White_charge_state);
		return 1;
		
	}  
	return 0;
}

/*アーム周り*/
int MCU_arm_control(int command, int limitTime){
    static int is_moving = 0;
	static uint32_t start_arm_control_time = 0;

	if(command != last_arm_command){
		is_moving = 0;
		start_arm_control_time = 0;
		running_arm_command = command;
		arm_position = 10;
	}

    if(is_moving == 0){ //command != arm_position && 
        uint8_t body[1] = {running_arm_command};
        DEBUG_PRINTF("send MCU arm command: %d\n\r", body[0]);
        ecan_sendPacketMtoU(&hcan1, 16, 2, 3, 0, 1, body);
        last_arm_command = running_arm_command;
        is_moving = 1;
		start_arm_control_time = HAL_GetTick();
    }

	//----
    if(is_moving == 1 && ((HAL_GetTick() - start_arm_control_time )<= limitTime)){
        if(data_type_MCU2[0] == 3 && data_type_MCU2[1] == 1 && data_MCU2[1] == running_arm_command){
            is_moving = 0;
            arm_position = running_arm_command;
			start_arm_control_time = 0;
            DEBUG_PRINTF("moving done \t current position:%d\n\r",arm_position);
            return 1;
        }else{
            //DEBUG_PRINTF("moving to position %d\n\r",command);
        }

    }else if(is_moving == 1 && ((HAL_GetTick() - start_arm_control_time ) > limitTime)){//timeout
		HAL_GPIO_TogglePin(BZ_GPIO_Port, BZ_Pin);
		is_moving = 0;
        arm_position = -10;
		start_arm_control_time = 0;
		DEBUG_PRINTF("time out\n\r");
		//HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
		HAL_GPIO_TogglePin(BZ_GPIO_Port, BZ_Pin);
		return 2;
	}
    return 0;
}

bool injection_set(void){//射出にセット 自動(アーム上に移動⇒装填) //d
	if(running_arm_command != injection_position){//さっきまでの
		if(Black_charge_state != 1 || White_charge_state != 1){
			charge_FLAG = 1; //チャージ開始
			
		} 
		DEBUG_PRINTF("running_arm_command != injection_position\n\r");

	}
	
	if(arm_position != injection_position){
		if(MCU_arm_control(injection_position, 4500))	return true;

	}else{
		if(Black_charge_state != 1 || White_charge_state != 1){
			charge_FLAG = 1; //チャージ開始
			
		} 
		return true;
	}

	// if(Black_charge_state == 1 && White_charge_state == 1 && arm_position == injection_position){
		// if(injection_set_start_time == 0) injection_set_start_time = HAL_GetTick();
		// if( (HAL_GetTick() - injection_set_start_time) >= release_timeout){
		// 	catch_open(1);
		// 	injection_set_start_time = 0;

		// 	return true;
	// 	}else{
	// 		return true;
	// }

	return false;
}

int arm_drag_set(int catch_maintain_time){
	static uint32_t arm_catch_timecount = 0;
	static int arm_drag_set_processNo = 0;
	static int move_result = 0;
	static int destination_position = 0;

	if(running_arm_command != destination_position){//さっきまでの
		arm_drag_set_processNo = 0;
		arm_catch_timecount  = 0;
		move_result = 0;


	}
	
	if(arm_drag_set_processNo == 0){
		if(arm_position == drag_position){
			if(catch_state == 0){//もしクリーナーを持っていなかったら　下で止まる
				arm_drag_set_processNo = 0;
				arm_catch_timecount  = 0;
				return 1;
			}else if(catch_state == 1){ //クリーナーを持っていたら離す
				catch_open(0);
				if(arm_catch_timecount == 0) arm_catch_timecount= HAL_GetTick();
				if((HAL_GetTick() - arm_catch_timecount) >= catch_maintain_time){
					catch_state = 0;
					arm_drag_set_processNo = 1;
					move_result = 0;
				}
			}
		}else{
			destination_position = drag_position;
			move_result = MCU_arm_control(destination_position, 4000);
			if(move_result == 2){
				arm_drag_set_processNo = 0;
				arm_catch_timecount  = 0;
				move_result = 0;
				return 2; //timeout
			} 
		}

	}else if(arm_drag_set_processNo == 1){
		if(arm_position == aim_position){
			arm_drag_set_processNo = 0;
			arm_catch_timecount  = 0;
			move_result = 0;
			return 1;
		}else{

			destination_position = aim_position;
			int move_result = MCU_arm_control(destination_position, 3000);
			if(move_result == 2){
				arm_drag_set_processNo = 0;
				arm_catch_timecount  = 0;
				move_result = 0;
				return 2; //timeout
			} 
		}
	}

	return 0;
}

bool injection_reload_from_drag(int catch_maintain_time){ //引きずり機構から自動装てん
	static uint32_t arm_drag_set_timecount = 0;
	static int arm_drag_set_processNo = 0;
	
	//DEBUG_PRINTF("%d\n\r",arm_drag_set_processNo);

	if(arm_drag_set_processNo == 0){//射出　→　引きずり →　キャッチ
		//DEBUG_PRINTF("arm_position: %d  catch_state:%d  timecount:%d\n\r",arm_position,catch_state,arm_drag_set_timecount);
		if(arm_position == drag_position){
			catch_close(1);  

			if(arm_drag_set_timecount == 0) arm_drag_set_timecount = HAL_GetTick();
			if((HAL_GetTick() - arm_drag_set_timecount) >= catch_maintain_time){
				arm_drag_set_processNo = 1;
				arm_drag_set_timecount = 0;
			}

		}
		if(catch_state == 1 && arm_position != drag_position){//閉じてしまっていたら開ける
			catch_open(1);
		}
		if(arm_position != drag_position){
			MCU_arm_control(drag_position, 4000);
		}

	}else if(arm_drag_set_processNo == 1){//→　装填
		if(arm_position != injection_position){
			MCU_arm_control(injection_position, 6000);
		}
		
		if( Black_charge_state != 1 || White_charge_state != 1){
			charge_FLAG = 1; //チャージ開始
		}

		if(arm_position == injection_position && Black_charge_state == 1 && White_charge_state == 1){
			catch_open(1);

			arm_drag_set_processNo = 0;
			arm_drag_set_timecount  = 0;
			DEBUG_PRINTF("END\n\r");
			return true;

		}
	}

	return false;

}

int send_calibration_signal(void){
	
	uint8_t calibration_body[1] = {1};
	ecan_sendPacketMtoU(&hcan1, 16, 2, 3, 2, 1, calibration_body);
	DEBUG_PRINTF("send calibration signal\n\r");
	return 1;
}


/*つかむ機構*/
void catch_open(int autoUpdate){
	RU_control(&hcan1, 1, catch_relay_port, 0);
	if(autoUpdate) catch_state = 0;
	return;
}

void catch_close(int autoUpdate){
	RU_control(&hcan1, 1, catch_relay_port, 1);
	if(autoUpdate) catch_state = 1;
	return;
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

bool getBtnHoldState(ButtonHandle *btn, uint32_t HOLD_TIME) {
    if (btn->BTN_state) {  // 押されている間は1
        if (!btn->is_pressed) {
            // 押し始めた瞬間
            btn->press_start_time = HAL_GetTick();
            btn->is_pressed = 1;
        } else {
            // 押し続けている → 経過時間チェック
            if ((HAL_GetTick() - btn->press_start_time) >= HOLD_TIME) {
                return true;  // HOLD_TIME以上押された！
            }
        }
    } else {
        // 離されたらリセット
        btn->is_pressed = 0;
    }
    return false;
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
		 ex: DEBUG_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
