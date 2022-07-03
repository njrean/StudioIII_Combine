/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENDEFF_ADDR 0x23<<1 //adress of eeprom
#define clockwise GPIO_PIN_SET
#define counterclockwise GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Run Motor
float volt = 0;
float volt_check = 0;

//Set Flag and Status
uint8_t SetHome_Flag = 0;
uint8_t AlSet_Flag = 2;
uint8_t Go_Flag = 0;
uint8_t TrajectoryGenerator_Flag = 0;
uint8_t GenVolt_Flag = 0;
uint8_t Emergency_status = 0;
uint8_t trigger = 0;

//ReadEncoder
float Rads = 0;
float angle = 0;
float angle_before = 0;
float theta_now = 0;			//angle sum of robot arm
float angle_sum_before = 0;
float angle_base_before = 0;
float angle_base = 0;
float threshold = 0.5*2*M_PI;
float angle_max =2*M_PI;

//Backward Difference
static float omega_tosensor = 0;

//Kalman Filter
uint8_t run_kalman = 0;
static float position_kalman = 0;
static float omega_kalman = 0;
float alpha_kalman = 0;
float jerk_kalman = 0;
float dt = 0.001;

arm_matrix_instance_f32 I;
arm_matrix_instance_f32 A;
arm_matrix_instance_f32 G;
arm_matrix_instance_f32 Q;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 R;

arm_matrix_instance_f32 input;
arm_matrix_instance_f32 y;
arm_matrix_instance_f32 y_old;
arm_matrix_instance_f32 x;
arm_matrix_instance_f32 x_new;
arm_matrix_instance_f32 P;
arm_matrix_instance_f32 P_new;

float32_t data_I[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
float32_t data_A[16];
float32_t data_G[4];
float32_t data_C[4] = {1,0,0,0};
float32_t data_R[1] = {0.1};
float32_t data_Q[1] = {100};
float32_t data_input[1] = {0};
float32_t data_K[4] = {0,0,0,0};
float32_t data_x[4] = {0,0,0,0};
float32_t data_x_new[4] = {0,0,0,0};
float32_t data_P[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float32_t data_P_new[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float32_t data_y[1] = {0};
float32_t data_y_old[1] = {0};

//Trajectory
	//Time at period
float t = 0;
float t1 = 0;
float t2 = 0;
float t3 = 0;
float t4 = 0;
float t5 = 0;
float t6 = 0;
float t7 = 0;
float tj = 0;
float ta = 0;
float tv = 0;
	//Variable of Trajectory
double w_max = M_PI/3;		//omega_ref max (rad/s)
double a_max = 0.3;			//alpha_ref max (rad/s^2)
double j_max = 0.5;			//jerk max	(rad/s^3)
double theta_0 = 0;			//theta_ref now (rad)
double theta_f = 0; 		//theta_ref want to go (rad)
double theta_dest = 0;		//theta_ref relative between theta_0 and theta_f (rad)

static float a[6];
static float v[6];
static float p[6];

	//Trajectory Reference
static double theta_ref = 0;
static double omega_ref = 0;
static float alpha_ref = 0;

//Cascade Position and Velocity
static float e1 = 0;
static float s1 = 0;
static float p1 = 0;
static float u1 = 0;

static float e2 = 0;
static float s2 = 0;
static float p2 = 0;
static float u2 = 0;

float kp_1 = 0;
float ki_1 = 0;
float kd_1 = 0;

float kp_2 = 0;
float ki_2 = 0;
float kd_2 = 0;

float kp_1_m = 0;
float ki_1_m = 0;
float kd_1_m = 0;

float kp_2_m = 0.002;
float ki_2_m = 0.04;
float kd_2_m = 0.00001;

float kp_1_l = 0;
float ki_1_l = 0;
float kd_1_l = 0;

float kp_2_l = 0;
float ki_2_l = 0.005;
float kd_2_l = 0.0008;

//Set Home
uint8_t SetZeroState = 1;

//UART Variable
uint16_t UARTDelay = 95;
uint8_t RxData[15];
uint8_t TxData[6] = {0x58, 0b01110101,0,0,0,0};
uint8_t TxData2[8] = {70, 110,0x58, 0b01110101,0,0,0,0};
uint8_t ack1[2] = {0x58, 0b01110101};

uint8_t Enable_EndEffector =0;

uint64_t _micro = 0;
uint8_t ModeN =0;
uint8_t ReachGoal =0;

uint8_t Finish =0;

//Station Main loop
float station[10] = {0,90,180,270,50,60,70,80,90,355};
uint8_t Current_station=1;
uint8_t index_station[16];
uint8_t n_station_max=1;
uint8_t n_station=1;

enum {Run, Home, EndEffector, Emergency, Main, Setzero, PrepareRun
}Arm_State = Main;

//EndEffector
uint8_t FlagOpen_EndEffector = 0;
uint8_t FlagRead_EndEffector = 0;
static uint16_t EndEffector_Status = 0;
uint32_t EndEffector_timestamp  = 0;

static enum
{
	State_start,
	State_open,
	State_shoot,
	State_close,
	State_wait
} EndEffector_State = State_wait;

//For Experiment
uint32_t timestamp = 0;
uint8_t dir = 1;
uint8_t PID_dir = 1;
//static uint8_t state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//Run Motor
void RunMotor(float volt, uint8_t direction);

//Set Home
void SetHome();

//Read Encoder and ReadEncoder
void ReadEncoder();

//Estimate omega_ref From Encoder's data to be sensor data in Kalman Filter
void BackwardDifference();

//Kalman Filter
void setmatrix();
void prediction();
void update();
void kalmanfilter();

//Make Trajectory
void TrajectoryGenerator();
void TrajectoryEvaluation();

//Cascade Position and Velocity Control
float PositionController(float r,float y);
float VelocityController(float r,float y,float uP);
float Cascade(float Pd,float P,float Vd,float V);

//Function For Operation
float positive(float var);
float negative(float var);
float limit(float var1, float var2);

//UI by UART Protocol
void UART();

//Microsec Timer
uint64_t micros();

//EndEffector
void OpenEndEffector();
void CheckEndEffector();

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Set Matrix for Kalman Filter
  HAL_Delay(200);
  setmatrix();

  //PWM start
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //Encoder start
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  //Timer to Read Encoder
  HAL_TIM_Base_Start_IT(&htim4);

  //Timer for micro
  HAL_TIM_Base_Start_IT(&htim11);

  //Close Yellow Pilot Lamp
  HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (Arm_State) {
		case Main:
			UART();
			ReadEncoder();
			kalmanfilter();
			break;
		case Home:
			UART();
			ReadEncoder();
			SetHome();
			kalmanfilter();
			break;
		case Emergency:
			UART();
			if(Emergency_status == 1){
				Arm_State = Emergency;
				Finish=1;
			}
			else {
				Arm_State = Main;
			}
			break;
		case Setzero:
			UART();
			if(micros() - timestamp > 2000000){
				if (SetZeroState)
				{
					TIM3->CNT = 0;
					angle_base = 0;
					angle = 0;
					theta_now = 0;
				}
				Arm_State = Main;
			 }

			break;
		case PrepareRun:
			UART();
			TrajectoryGenerator_Flag=1;
			TrajectoryGenerator();
			Arm_State = Run;
			break;
		case Run:
			UART();
			if(Go_Flag == 0){
				if(ModeN==1){
					if(n_station >= n_station_max-1){
						Arm_State = EndEffector;
						FlagOpen_EndEffector =1;
						ModeN=0;
						Finish = 1;
					}
					else{
						n_station++;
					}
					Arm_State = EndEffector;
					FlagOpen_EndEffector =1;
					theta_f = station[index_station[n_station]-1]*(M_PI/180.0);
				}
				else{
					Arm_State = EndEffector;
					FlagOpen_EndEffector =1;
					Finish =1;
				}
			}
			break;
		case EndEffector:
			UART();
			if(Enable_EndEffector == 1) //Enable Effector
			{
				OpenEndEffector();
				if(EndEffector_State == State_wait){
					if(ModeN==1){
						Arm_State = PrepareRun;
					}
					else{
						Arm_State = Main;
					}
				}
			}

			else // Disable Effector
			{
				FlagOpen_EndEffector =0;
				if(ModeN==1){
					Arm_State = PrepareRun;
				}
				else{
					Arm_State = Main;
				}
			}
			break;

	}
	 /*switch(state)
	 {
	 case Main:
		 UART();
		 Finish = 0;
		 ReadEncoder();
		 break;
	 case Run:
		 UART();
		 if(ModeN==0){
			 if(Go_Flag==0){
				 TrajectoryGenerator_Flag=1;
				 TrajectoryGenerator();
			 }

			 else if(Go_Flag==1){
				 if(theta_ref == theta_f){
					 Go_Flag =0;
					 volt = 0;
					 RunMotor(volt, PID_dir);
					 if(Enable_EndEffector==1){
						 FlagOpen_EndEffector=1;
					 }
					 else{
						 FlagOpen_EndEffector=0;
					 }
					 state = EndEffector;
				 }
			 }
		 }
		 else if(ModeN ==1){
			 if(n_station > n_station_max){
				 Go_Flag=0;
				 Finish=1;
				 state = Main;
			 }
			 else if(Go_Flag==0){
				 TrajectoryGenerator_Flag=1;
				 theta_f = station[index_station[n_station]];
				 TrajectoryGenerator();
				 n_station++;
			 }
			 else if(Go_Flag==1){
				 if(theta_ref == theta_f){
					 Go_Flag=0;
					 volt = 0;
					 RunMotor(volt, PID_dir);
					 if(Enable_EndEffector==1){
						 FlagOpen_EndEffector=1;
					 }
					 else{
						 FlagOpen_EndEffector=0;
					 }
					 state = EndEffector;
				 }
			 }
		 }
		 break;
	 case Home:
		 Finish=0;
		 UART();
		 SetHome();
		 ReadEncoder();
		 BackwardDifference();
		 kalmanfilter();
		 break;
	 case Setzero:
		 UART();
		 if(micros() - timestamp > 2000000){
			 TIM3->CNT = 0;
			theta_now = 0;
			state = Main;
			Finish=0;
		 }
		 break;
	 case EndEffector:
		 UART();
		 OpenEndEffector();
		 if(ModeN==0 && EndEffector_State == State_wait ){
			 state= Main;
			 Finish=1;
		 }
		 else if(ModeN==1 && EndEffector_State == State_wait){
			 state= Run;
		 }
		 break;
	 case Emergency:
		 if(Emergency_status==0){
			 state=Main;
		 }
		 break;
	 default:
		 state =Main;
		 break;
	 }*/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8191;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor_DIR_Pin|PilotLamp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Proximity_Pin */
  GPIO_InitStruct.Pin = Proximity_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Proximity_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_DIR_Pin */
  GPIO_InitStruct.Pin = Motor_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergency_Pin */
  GPIO_InitStruct.Pin = Emergency_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Emergency_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PilotLamp_Pin */
  GPIO_InitStruct.Pin = PilotLamp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PilotLamp_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void RunMotor(float volt, uint8_t direction)
{
	static float PWMOut = 0;
	PWMOut = (volt*5000.0)/24.0;
	HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, direction);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMOut);
}

void ReadEncoder()
{
//	angle_sum_before = theta_now;
//	theta_now = (TIM3->CNT/8191.0)*(2.0*M_PI);

	angle_before = angle;
	angle_sum_before = theta_now;

	angle = (TIM3->CNT/8191.0)*(2.0*M_PI);
	angle_base_before = angle_base;
	if ((angle - angle_before) <= -threshold)
	{
		angle_base = angle_base_before + angle_max;
	}
	else if ((angle - angle_before) >= threshold)
	{
		angle_base = angle_base_before - angle_max;
	}
	else
	{
		angle_base = angle_base_before;
	}

	theta_now = angle + angle_base;
}

void BackwardDifference()
{
	omega_tosensor = (theta_now-angle_sum_before)/dt;
}

void SetHome()
{
	if (HAL_GPIO_ReadPin(Proximity_GPIO_Port, Proximity_Pin) == GPIO_PIN_RESET && AlSet_Flag == 2)
	{
		SetHome_Flag = 2;
	}

	if(SetHome_Flag == 1)
	{
		volt = 9;
		RunMotor(volt, counterclockwise);
		AlSet_Flag = 1;
		SetHome_Flag = 0;
	}

	else if(SetHome_Flag == 2)
	{
		volt = 6.5;
		RunMotor(volt, counterclockwise);
		AlSet_Flag = 0;
		SetHome_Flag = 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Proximity_Pin && AlSet_Flag == 0)
	{
		volt = 0;
		RunMotor(volt, clockwise);
		theta_now = 0;
		kalmanfilter();
		AlSet_Flag = 2;
		timestamp = micros();
		Arm_State = Setzero;
	}

	else if (GPIO_Pin == Proximity_Pin && AlSet_Flag == 1)
	{
		SetHome_Flag = 2;
	}

	if (GPIO_Pin == B1_Pin)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		SetZeroState = (SetZeroState+1)%2;
	}

	if (GPIO_Pin == Emergency_Pin)
	{
		if (HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_SET)
		{
			Arm_State=Main;
			Finish = 1;
			Emergency_status = 1;
			HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_SET);
			volt = 0;
			RunMotor(volt, clockwise);
		}
		else
		{
			Emergency_status = 0;
		}
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4 && Go_Flag && Arm_State == Run)
	{
		ReadEncoder();
		BackwardDifference();
		TrajectoryEvaluation();
		kalmanfilter();

		volt = Cascade(theta_ref, position_kalman, omega_ref, omega_kalman);

		t+=dt;


		if (Go_Flag == 0) //when last loop
		{
			volt = 0;
			t = 0;
		}


		RunMotor(volt, PID_dir);
	}

	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

void setmatrix(){
	// A = {1,dt,0.5*dt*dt,0,1,dt,0,0,1};
	data_A[0]=1;
	data_A[1]=dt;
	data_A[2]=dt*dt/2.0;
	data_A[3]=dt*dt*dt/6.0;
	data_A[4]=0;
	data_A[5]=1;
	data_A[6]=dt;
	data_A[7]=dt*dt/2.0;
	data_A[8]=0;
	data_A[9]=0;
	data_A[10]=1;
	data_A[11]=dt;
	data_A[12]=0;
	data_A[13]=0;
	data_A[14]=0;
	data_A[15]=1;

	// G = {dt*dt*dt/6,dt*dt/2,dt,1};
	data_G[0]=dt*dt*dt/6.0;
	data_G[1]=dt*dt/2.0;
	data_G[2]=dt;
	data_G[3]=1;

	arm_mat_init_f32(&I, 4, 4, data_I);
	arm_mat_init_f32(&A, 4, 4, data_A);
	arm_mat_init_f32(&G, 4, 1, data_G);
	arm_mat_init_f32(&C, 1, 4, data_C);
	arm_mat_init_f32(&R, 1, 1, data_R);
	arm_mat_init_f32(&Q, 1, 1, data_Q);

	arm_mat_init_f32(&input, 1, 1, data_input);
	arm_mat_init_f32(&K, 4, 1, data_K);
	arm_mat_init_f32(&x, 4, 1, data_x);
	arm_mat_init_f32(&x_new, 4, 1, data_x_new);
	arm_mat_init_f32(&P, 4, 4, data_P);
	arm_mat_init_f32(&P_new, 4, 4, data_P_new);
	arm_mat_init_f32(&y, 1, 1, data_y);
	arm_mat_init_f32(&y_old, 1, 1, data_y_old);
}

void prediction(){

	arm_mat_mult_f32(&A, &x, &x_new);  //x_new = multiply(A, x);

	//P_new = sum(multiply(multiply(A, P), transpose(A)), multiply(multiply(G, Q), transpose(G)));
	float32_t data_mult1[16];
	arm_matrix_instance_f32 mult1;
	arm_mat_init_f32(&mult1, 4, 4, data_mult1);

	float32_t data_mult2[4];
	arm_matrix_instance_f32 mult2;
	arm_mat_init_f32(&mult2, 4, 1, data_mult2);

	float32_t data_mult3[16];
	arm_matrix_instance_f32 mult3;
	arm_mat_init_f32(&mult3, 4, 4, data_mult3);

	float32_t data_A_T[16];
	arm_matrix_instance_f32 A_T;
	arm_mat_init_f32(&A_T, 4, 4, data_A_T);

	float32_t data_G_T[4];
	arm_matrix_instance_f32 G_T;
	arm_mat_init_f32(&G_T, 1, 4, data_G_T);

	arm_mat_trans_f32(&A, &A_T);
	arm_mat_trans_f32(&G, &G_T);
	arm_mat_mult_f32(&A, &P, &mult1);
	arm_mat_mult_f32(&G, &Q, &mult2);
	arm_mat_mult_f32(&mult1, &A_T, &mult1);
	arm_mat_mult_f32(&mult2, &G_T, &mult3);
	arm_mat_add_f32(&mult1, &mult3, &P_new);
}

void update(){
	float32_t data_sumK[1];
	arm_matrix_instance_f32 sumK;
	arm_mat_init_f32(&sumK, 1, 1, data_sumK);

	float32_t data_mult3x3[16];
	arm_matrix_instance_f32 mult3x3;
	arm_mat_init_f32(&mult3x3, 4, 4, data_mult3x3);

	float32_t data_mult3x1[4];
	arm_matrix_instance_f32 mult3x1;
	arm_mat_init_f32(&mult3x1, 4, 1, data_mult3x1);

	float32_t data_mult1x3[4];
	arm_matrix_instance_f32 mult1x3;
	arm_mat_init_f32(&mult1x3, 1, 4, data_mult1x3);

	float32_t data_mult1x1[1];
	arm_matrix_instance_f32 mult1x1;
	arm_mat_init_f32(&mult1x1, 1, 1, data_mult1x1);

	float32_t data_C_T[4];
	arm_matrix_instance_f32 C_T;
	arm_mat_init_f32(&C_T, 4, 1, data_C_T);

	//sumK = sum(R, multiply(multiply(C, P_new), transpose(C)));
	arm_mat_trans_f32(&C, &C_T);
	arm_mat_mult_f32(&C, &P_new, &mult1x3);
	arm_mat_mult_f32(&mult1x3, &C_T, &mult1x1);
	arm_mat_add_f32(&R, &mult1x1, &sumK);

	arm_matrix_instance_f32 I_sumK;
	float32_t data_I_sumK[1] = {1/data_sumK[0]};
	arm_mat_init_f32(&I_sumK, 1, 1, data_I_sumK);

	//K = multiply(multiply(P_new, transpose(C)), I_sumK);
	arm_mat_mult_f32(&P_new, &C_T, &mult3x1);
	arm_mat_mult_f32(&mult3x1, &I_sumK, &K);

	//P = multiply(minus(I, multiply(K, C)), P_new);
	arm_mat_mult_f32(&K, &C, &mult3x3);
	arm_mat_sub_f32(&I, &mult3x3, &mult3x3);
	arm_mat_mult_f32(&mult3x3, &P_new, &P);

	//data_input[0] = theta_now;
	data_input[0] = theta_now;

	//y_old = multiply(C, x_new);
	arm_mat_mult_f32(&C, &x_new, &y_old);

	//y = minus(input, y_old);
	arm_mat_sub_f32(&input, &y_old, &y);

	//x = sum(multiply(K, y), x_new);
	arm_mat_mult_f32(&K, &y, &mult3x1);
	arm_mat_add_f32(&mult3x1, &x_new, &x);
}

void kalmanfilter()
{
	prediction();
	update();
	position_kalman = data_x_new[0];
	omega_kalman = data_x_new[1];
	alpha_kalman = data_x_new[2];
	jerk_kalman = data_x_new[3];
}

void TrajectoryGenerator()
{	if(TrajectoryGenerator_Flag)
	{
		//Variable For Consider Case
		static float M;
		static float N;
		static float Va;
		static float Sa;
		static float Sv;

		j_max = positive(j_max);
		a_max = positive(a_max);
		w_max = positive(w_max);

		theta_0 = theta_now;

		theta_dest = theta_f - theta_0;

		if(theta_dest < 0)
		{
			theta_dest = -(theta_dest);
			dir = 0;
		}

		else
		{
			dir = 1;
		}

		if(w_max*j_max < pow(a_max,2.0)){
			M=1.0;
			N=0.0;
		}

		else if(w_max*j_max >= pow(a_max,2.0)){
			M=0.0;
			N=1.0;
		}

		Va = pow(a_max,2.0)/j_max;
		Sa = (2.0*pow(a_max,3.0))/(pow(j_max,2.0));
		Sv = w_max*(M*(2.0*sqrt(w_max/j_max))+N*((w_max/a_max)+(a_max/j_max)));

		if(w_max < Va){
			if(theta_dest > Sa){
				//caseI
				tj = sqrt((w_max/j_max));
				ta = tj;
				tv = theta_dest/w_max;
			}
			else if(theta_dest < Sa){
				if(theta_dest < Sv){
					//caseIV
					tj = pow((theta_dest/(2.0*j_max)),0.3333);
					ta = tj;
					tv = 2.0*tj;
				}
				else if(theta_dest > Sv){
					//caseIII
					tj = sqrt((w_max/j_max));
					ta = tj;
					tv = theta_dest/w_max;
				}
			}
		}
		else if(w_max >= Va){
			if(theta_dest < Sa){
				//caseII
				tj = pow((theta_dest/(2.0*j_max)),0.3333);
				ta = tj;
				tv = 2.0*tj;
			}
			else if(theta_dest >= Sa){
				if(theta_dest < Sv){
					//caseVI
					tj = a_max/j_max;
					ta = 0.5*(sqrt(((4.0*theta_dest*pow(j_max,2.0))+pow(a_max,3.0))/(a_max*pow(j_max,2.0)))-(a_max/j_max));
					tv = ta + tj;
				}
				else if(theta_dest >= Sv){
					//caseV
					tj = a_max/j_max;
					ta = w_max/a_max;
					tv = theta_dest/w_max;
				}
			}
		}

		t1 = tj;
		t2 = ta;
		t3 = ta + tj;
		t4 = tv;
		t5 = tv + tj;
		t6 = tv + ta;
		t7 = tv + tj + ta;

		theta_ref = theta_0;
		omega_ref = 0;
		alpha_ref = 0;

		if (dir == 0)
		{
			j_max = negative(j_max);
			a_max = negative(a_max);
			w_max = negative(w_max);
		}

		else
		{
			j_max = positive(j_max);
			a_max = positive(a_max);
			w_max = positive(w_max);
		}

		p[0] = (1.0/6.0)*j_max*pow(t1,3.0);
		v[0] = 0.5*j_max*pow(t1,2.0);
		a[0] = j_max*t1;

		p[1] = p[0] + v[0]*(t2-t1) + 0.5*a[0]*pow((t2-t1),2.0);
		v[1] = v[0] + a[0]*(t2-t1);
		a[1] = a[0];

		p[2] = p[1] + v[1]*(t3-t2) + 0.5*a[1]*pow((t3-t2),2.0) - j_max*pow((t3-t2),3.0)/6.0;
		v[2] = v[1] + a[1]*(t3-t2) - 0.5*j_max*pow((t3-t2),2.0);
		a[2] = a[1] - j_max*(t3-t2);

		p[3] = p[2] + v[2]*(t4-t3);
		v[3] = v[2];
		a[3] = a[2];

		p[4] = p[3] + v[3]*(t5-t4) - j_max*pow((t5-t4),3.0)/6.0;
		v[4] = v[3] - 0.5*j_max*pow((t5-t4),2.0);
		a[4] = a[3]- j_max*(t5-t4);

		p[5] = p[4] + v[4]*(t6-t5) + 0.5*(a[4])*pow((t6-t5),2.0);
		v[5] = v[4] + a[4]*(t6-t5);
		a[5] = a[4];

		//Change Cascade Gain
		if (theta_dest > (2*M_PI/9))
		{
			kp_1 = kp_1_m;
			ki_1 = ki_1_m;
			kd_1 = kd_1_m;

			kp_2 = kp_2_m;
			ki_2 = ki_2_m;
			kd_2 = kd_2_m;

		}

		else
		{
			kp_1 = kp_1_l;
			ki_1 = ki_1_l;
			kd_1 = kd_1_l;

			kp_2 = kp_2_l;
			ki_2 = ki_2_l;
			kd_2 = kd_2_l;
		}


		//Set Flag to Go
		TrajectoryGenerator_Flag = 0;
		Go_Flag = 1;

		//Reset Cascade PID
		e1 = 0;
		s1 = 0;
		p1 = 0;
		u1 = 0;
		e2 = 0;
		s2 = 0;
		p2 = 0;
		u2 = 0;
	}
}

void TrajectoryEvaluation()
{
	if( 0 <= t && t < t1){
		theta_ref = theta_0 + (1.0/6.0)*j_max*pow(t,3.0);
		omega_ref = 0.5*j_max*pow(t,2.0);
		alpha_ref = j_max*t;
	}
	else if (t1 <= t && t< t2){
		theta_ref = theta_0 + p[0] + v[0]*(t-t1) + 0.5*a[0]*pow((t-t1),2.0);
		omega_ref = v[0] + a[0]*(t-t1);
		alpha_ref = a[0];
	}
	else if (t2 <= t && t < t3){
		theta_ref = theta_0 + p[1] + v[1]*(t-t2) + 0.5*a[1]*pow((t-t2),2.0) - j_max*pow((t-t2),3.0)/6.0;
		omega_ref = v[1] + a[1]*(t-t2) - 0.5*j_max*pow((t-t2),2.0);
		alpha_ref = a[1] - j_max*(t-t2);
	}
	else if (t3 <= t && t < t4 ){
		theta_ref = theta_0 + p[2] + v[2]*(t-t3);
		omega_ref = v[2];
		alpha_ref = 0;
	}
	else if (t4 <= t && t < t5 ){
		theta_ref = theta_0 + p[3] + v[3]*(t-t4) - j_max*pow((t-t4),3.0)/6.0;
		omega_ref = v[3] - 0.5*j_max*pow((t-t4),2.0);
		alpha_ref = -j_max*(t-t4);
	}
	else if (t5 <= t && t < t6 ){
		theta_ref = theta_0 + p[4] + v[4]*(t-t5) + 0.5*a[4]*pow((t-t5),2.0);
		omega_ref = v[4] + a[4]*(t-t5);
		alpha_ref = a[4];
	}
	else if (t6 <= t && t < t7 ){
		theta_ref = theta_0 + p[5] + v[5]*(t-t6) + 0.5*a[5]*pow((t-t6),2.0) + j_max*pow((t-t6),3.0)/6.0;
		omega_ref = v[5] + a[5]*(t-t6) + 0.5*j_max*pow((t-t6),2.0);
		alpha_ref = a[5] + j_max*(t-t6);
	}
	else if (t7 <= t ){
		theta_ref = theta_f;
		omega_ref = omega_ref;
		alpha_ref = alpha_ref;

		if (dir == 1 && theta_now >= (theta_ref - 0.008) && theta_now <= theta_ref)
		{
			volt = 0;
			RunMotor(volt, PID_dir);
			Go_Flag = 0;
			t = 0;
		}

		else if (dir == 0 && theta_now >= theta_ref && theta_now <= (theta_ref + 0.008))
		{
			volt = 0;
			RunMotor(volt, PID_dir);
			Go_Flag = 0;
			t = 0;
		}
	}
}

float PositionController(float r,float y) //r == trajectory, y==feedback
{
	e1 = r - y;
	s1 = s1 + e1;
	u1 = (kp_1*e1) + (ki_1*s1) + (kd_1*(e1-p1));
	p1 = e1;
	return u1;
}

float VelocityController(float r,float y,float uP) //r == trajectory, y==feedback
{
	e2 = uP + r;

	if (dir == 0 && e2 <= w_max)
	{
		e2 = w_max;
	}

	else if (dir == 1 && e2 >= w_max)
	{
		e2 = w_max;
	}

	e2 = e2 - y;
	s2 = s2 + e2;
	u2 = (kp_2*e2) + (ki_2*s2) + (kd_2*(e2-p2));
	p2 = e2;
	return u2;
}

float Cascade(float Pd,float P,float Vd,float V){
	static float u;
	static float add = 0;
	u = PositionController(Pd, P);
	u = VelocityController(Vd, V, u);
	if (u >= 0)
	{
		PID_dir = 1;
	}

	else
	{
		u = -u;
		PID_dir = 0;
	}


	//add = (6.2*(t/t1) < 6.2) ? 6.2*(t/t1) : 6.2;

//	if(t >= t5 && t < t6)
//	{
//		add = 4.7-((1.6/(t6-t5))*(t-t5));
//	}
//
//	else if (t >= t6)
//	{
//		add = 4.7-1.6;
//	}

	return limit(u, add);
}

float negative(float var)
{
	if (var > 0){
		var = -var;
	}
	return var;
}

float positive(float var)
{
	if (var < 0){
			var = -var;
		}
	return var;
}

float limit(float var1, float var2)
{
	return (var1 > 24.0-var2) ? 24.0 : var1+var2;

}

void UART(){

	static uint8_t HighByte =0;
	static uint8_t LowByte = 0;
	static uint16_t DataByte =0;
	static uint8_t CheckTrasmit=1;
	static uint32_t Timestamp_UI =0;

	switch(RxData[0])
	{
	case 0b10010001: //Go_Mode 1

		break;
	case 0b10010010: //Go_Mode2 FRAME#1
		if(RxData[1] == 0b01101101){
			//connect MCU
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);

		}
		break;
	case 0b10010011: //Go_Mode 3 FRAME#1
		if(RxData[1] == 0b01101100){
			//disconnect MCU
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10010100: //Go_Mode 4 FRAME#2
		HighByte = RxData[1];
		LowByte = RxData[2];
		DataByte = (HighByte<<8) + LowByte;
		if(RxData[3] == (uint8_t)(~(0b10010100 + HighByte + LowByte))){
			//Set Angular Velocity
			w_max = (double)DataByte*(10.0/255.0)*(2.0*M_PI/60.0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10010101: //Go_Mode 5 FRAME#2
		HighByte = RxData[1];
		LowByte = RxData[2];
		DataByte = (HighByte<<8) + LowByte;
		if(RxData[3] == (uint8_t)(~(0b10010101 + HighByte + LowByte))){
			//Set Angular Position
			theta_f = (double)DataByte/10000.0;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			ModeN =0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10010110: //Go_Mode 6 FRAME#2
		HighByte = RxData[1];
		LowByte = RxData[2];
		DataByte = (HighByte<<8) + LowByte;
		if(RxData[3] == (uint8_t)(~(0b10010110 + HighByte + LowByte))){
			//Set Goal 1 Station
			index_station[0] = LowByte;
			theta_f = station[index_station[0]-1]*(M_PI/180.0);
			ModeN =0;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10010111: //Go_Mode 7 FRAME#3
		n_station_max = RxData[1];
		uint8_t Check=0;
		uint8_t k=2;
		for(int j = 0;j<n_station_max;j+=2){
			index_station[j] = RxData[k]%16;
			index_station[j+1] = (uint8_t)RxData[k]/16;
			k+=1;
		}
		for(int f = 0; f<k;f++){
			Check += RxData[f];
		}
		if(RxData[k] == (uint8_t)(~(Check)) ){

			ModeN=1;
			n_station=0;
			theta_f = station[index_station[n_station]-1]*(M_PI/180.0);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10011000: //Go_Mode 8 FRAME#1
		if(RxData[1] == 0b01100111){
			//Go to Station / Goal Position
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			Arm_State = PrepareRun;
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10011001: //Go_Mode 9 FRAME#1
		if(RxData[1] == 0b01100110){
			//Request Current Station
			if(Finish){
				if(CheckTrasmit){
					TxData2[4]=TxData[2];
					TxData2[5]=TxData[3];
					TxData2[6]=TxData[4];
					TxData2[7]=TxData[5];
					CheckTrasmit =0;
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData2, 8); //send ack#1
					Timestamp_UI=micros();
				}
				else{
					if(micros() - Timestamp_UI > UARTDelay){
						RxData[0] = 0;
						CheckTrasmit=1;
						Finish =0;
						HAL_UART_DMAStop(&huart2);
					}
				}

			}
			else if(CheckTrasmit){
				TxData[2] = 0b10011001;
				TxData[3] = 0;
				Current_station = (uint8_t)ceil(theta_now/0.087); // 1 station = 0.087 rads ->72 station
				TxData[4] = Current_station;
				TxData[5] = (uint8_t)(~(TxData[2] + TxData[3] + TxData[4]));
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData, 6);
				Timestamp_UI=micros();
				CheckTrasmit=0;
			}
			else{
				if(micros() - Timestamp_UI > UARTDelay){
					RxData[0] = 0;
					HAL_UART_DMAStop(&huart2);
					CheckTrasmit=1;
				}
			}
		}
		break;
	case 0b10011010: //Go_Mode 10 FRAME#1
		if(RxData[1] == 0b01100101){
			//Request Angular Position
		if(Finish){

			if(CheckTrasmit){
				TxData2[4]=TxData[2];
				TxData2[5]=TxData[3];
				TxData2[6]=TxData[4];
				TxData2[7]=TxData[5];
				CheckTrasmit =0;
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData2, 8); //send ack#1
				Timestamp_UI=micros();
			}
			else{
				if(micros() - Timestamp_UI > UARTDelay){
					RxData[0] = 0;
					CheckTrasmit=1;
					Finish =0;
					HAL_UART_DMAStop(&huart2);
				}
			}
		}
		else if(CheckTrasmit){
			TxData[2] = 0b10011010;
			TxData[3] = (uint8_t)((theta_now*10000.0)/256.0);
			TxData[4] = (uint8_t)(theta_now*10000)%256;
			TxData[5] = (uint8_t)(~(TxData[2] + TxData[3] + TxData[4]));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData, 6);
			Timestamp_UI=micros();
			CheckTrasmit=0;
		}
		else{
			if(micros() - Timestamp_UI > UARTDelay){
				RxData[0] = 0;
				HAL_UART_DMAStop(&huart2);
				CheckTrasmit=1;
			}
		}

		}
		break;
	case 0b10011011: //Go_Mode 11 FRAME#1
		if(RxData[1] == 0b01100100){
			//Request Angular Velocity
		if(Finish){
			if(CheckTrasmit){
				TxData2[4]=TxData[2];
				TxData2[5]=TxData[3];
				TxData2[6]=TxData[4];
				TxData2[7]=TxData[5];
				CheckTrasmit =0;
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData2, 8); //send ack#1
				Timestamp_UI=micros();
			}
			else{
				if(micros() - Timestamp_UI > UARTDelay){
					RxData[0] = 0;
					CheckTrasmit=1;
					Finish =0;
					HAL_UART_DMAStop(&huart2);
					CheckTrasmit=1;
				}
			}
		}
		else if (CheckTrasmit){

			TxData[2] = 0b10011011;
			TxData[3] = 0;
			TxData[4] = (uint8_t)((positive(omega_kalman)*60/(2*M_PI))*255/10);
			TxData[5] = (uint8_t)(~(TxData[2] + TxData[3] + TxData[4]));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)TxData, 6);
			Timestamp_UI=micros();
			CheckTrasmit=0;
		}
		else{
			if(micros() - Timestamp_UI > UARTDelay){
				RxData[0] = 0;
				HAL_UART_DMAStop(&huart2);
				CheckTrasmit=1;
			}
		}

		}
		break;
	case 0b10011100: //Go_Mode 12 FRAME#1
		if(RxData[1]==0b01100011){
			//enable gripple
			Enable_EndEffector = 1;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10011101: //Go_Mode 13 FRAME#1
		if(RxData[1] == 0b01100010){
			//disable gripple
			Enable_EndEffector = 0;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0b10011110: //Go_Mode 14 FRAME#1
		if(RxData[1] == 0b01100001){
			//Set HOME
			Arm_State = Home;
			theta_now = 0.1;
			SetHome_Flag=1;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ack1, 2); //send ack#1
			RxData[0] = 0;
			HAL_UART_DMAStop(&huart2);
		}
		break;
	case 0:
		HAL_UART_Receive_DMA(&huart2, RxData, 12);
		break;
	case 88:
		if(RxData[0] == 'X' && RxData[1] == 'u'){
			if(RxData[2] !=0 && RxData[3] != 0){
				RxData[0]=RxData[2];
				RxData[1]=RxData[3];
				RxData[2] =0;
				RxData[3] =0;
				RxData[4] =0;
				RxData[5] =0;
			}
		}
		break;
	default: //RESET Go_Mode
		RxData[0] = 0;
		HAL_UART_DMAStop(&huart2);
		break;
	}
}

void OpenEndEffector() {
	if (hi2c1.State == HAL_I2C_STATE_READY && FlagOpen_EndEffector == 1)
	{
		static uint8_t addr_open = 0x45;
		HAL_I2C_Master_Transmit(&hi2c1, ENDEFF_ADDR, &addr_open, 1,100);
		FlagOpen_EndEffector = 0;
		FlagRead_EndEffector = 1;
		EndEffector_State = State_start;
		EndEffector_timestamp = HAL_GetTick();
	}

	if (FlagRead_EndEffector == 1)
	{
		if (HAL_GetTick() - EndEffector_timestamp > 250)
		{
			EndEffector_timestamp = HAL_GetTick();
			CheckEndEffector();
		}

		switch(EndEffector_State)
		{
		case State_start:
			if (EndEffector_Status == 0x12)
			{
				EndEffector_State = State_open;
			}
			break;
		case State_open:
			if (EndEffector_Status == 0x34)
			{
				EndEffector_State = State_shoot;
			}
			break;
		case State_shoot:
			if (EndEffector_Status == 0x56)
			{
				EndEffector_State = State_close;
			}
			break;
		case State_close:
			if (EndEffector_Status == 0x78)
			{
				EndEffector_State = State_wait;
			}
			break;
		case State_wait:
			FlagRead_EndEffector = 0;
			break;
		}
	}
}

void CheckEndEffector()
{
	static uint8_t addr = 0x23;
	HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, ENDEFF_ADDR, &addr, 1, I2C_FIRST_FRAME);
	if(hi2c1.State == HAL_I2C_STATE_READY)
	{
		HAL_I2C_Master_Seq_Receive_IT(&hi2c1, ENDEFF_ADDR, &EndEffector_Status, 1, I2C_LAST_FRAME);
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
