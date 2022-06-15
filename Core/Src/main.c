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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Run Motor
float volt = 0;

//Set Flag and Status
uint8_t SetHome_Flag = 0;
uint8_t AlSet_Flag = 1;
uint8_t Go_Flag = 0;
uint8_t TrajectoryGenerator_Flag = 0;
uint8_t GenVolt_Flag = 0;
uint8_t Emergency_status = 0;
uint8_t trigger = 0;

//Unwrap
float Rads = 0;
float angle = 0;
float angle_before = 0;
float angle_sum = 0;			//angle sum of robot arm
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
float32_t data_C[4] = {0,1,0,0};
float32_t data_R[1] = {14500};
float32_t data_Q[1] = {1};
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
double w_max = M_PI/3;		//omega max (rad/s)
double a_max = 0.5;			//alpha max (rad/s^2)
double j_max = 0.5;			//jerk max	(rad/s^3)
double theta_0 = 0;			//theta now (rad)
double theta_f = 3.0*M_PI/2.0;	//theta want to go (rad)
double theta_dest = 0;		//theta relative between theta_0 and theta_f (rad)

	//Trajectory Reference
static double theta = 0;
static double omega = 0;
static float alpha = 0;

//Cascade Position and Velocity
static float e1 = 0;
static float s1 = 0;
static float p1 = 0;
static float u1 = 0;

static float e2 = 0;
static float s2 = 0;
static float u2 = 0;

float kp_1 = 0.000001;
float ki_1 = 0.0000056;
float kd_1 = 0;

float kp_2 = 0.005;
float ki_2 = 0.03;


//For Experiment
uint32_t timestamp = 0;
uint8_t dir = 1;
uint8_t PID_dir = 1;
static uint8_t state = 0;
static float tt = 0;

enum {Defualt, RunSetHome, WaitSetHome, FinishSetHome, PreparePID, RunPID, FinishPID
}ArmState = Defualt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//Run Motor
void RunMotor(uint8_t volt, uint8_t direction);

//Set Home
void SetHome();

//Read Encoder and Unwrap
void Unwrap();

//Estimate omega From Encoder's data to be sensor data in Kalman Filter
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

//volt Generator for Experiment Tune Kalman Filter
void genvolt();

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  setmatrix();
  //PWM start
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //Encoder start
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  //Timer to Read Encoder
  HAL_TIM_Base_Start_IT(&htim4);

  //Set Matrix for Kalman Filter

  //
  HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_SET);
  SetHome_Flag = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 switch (ArmState)
	 {
	 	 case Defualt:

	 		 if (SetHome_Flag)
	 		 {
	 			 ArmState = RunSetHome;
	 			 break;
	 		 }

	 		 else if (Go_Flag)
	 		 {
	 			 ArmState = PreparePID;
	 			 break;
	 		 }

	 		 Unwrap();
	 		 RunMotor(volt, dir);
	 		 break;

	 	 case RunSetHome:
	 		 SetHome();
	 		 ArmState = WaitSetHome;
	 		 break;

	 	 case WaitSetHome:
	 		 Unwrap();
	 		 break;

	 	 case FinishSetHome:
	 		 HAL_Delay(2000);
	 		 TIM3->CNT = 0;
	 		 angle_before = 0;
	 		 angle = 0;
	 		 angle_base_before = 0;
	 		 angle_base =0;
	 		 angle_sum_before = 0;
	 		 angle_sum = 0;
	 		 Go_Flag = 1; //go!!
	 		 //GenVolt_Flag = 1;//gen volt
	 		 ArmState = Defualt;
	 		 break;

	 	 case PreparePID:
	 		 TrajectoryGenerator_Flag = 1;
	 		 TrajectoryGenerator();
	 		 TrajectoryGenerator_Flag = 0;
	 		 e1 = 0;
	 		 s1 = 0;
	 		 p1 = 0;
	 		 u1 = 0;
	 		 e2 = 0;
	 		 s2 = 0;
	 		 u2 = 0;
	 		 ArmState = RunPID;
	 		 break;

	 	 case RunPID:
	 		break;

	 	 case FinishPID:
	 		 volt = 0;
	 		 ArmState = Defualt;
	 		 break;
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

  /*Configure GPIO pin : Encoder_X_Pin */
  GPIO_InitStruct.Pin = Encoder_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Encoder_X_GPIO_Port, &GPIO_InitStruct);

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
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void RunMotor(uint8_t volt, uint8_t direction)
{
	static float PWMOut = 0;
	if (Emergency_status == 1)
	{
		volt = 0;
	}

	if (volt == 0)
	{
		HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_SET);
	}

	else
	{
		HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_RESET);
	}

	PWMOut = (volt*5000)/24;
	HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, direction);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMOut);
}

void Unwrap()
{
	angle_before = angle;
	angle_sum_before = angle_sum;

	angle = (TIM3->CNT/8191.0)*(2*M_PI);
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

	angle_sum = angle + angle_base;
}

void BackwardDifference()
{
	omega_tosensor = (angle_sum-angle_sum_before)/dt;
}

void SetHome()
{
	if(SetHome_Flag == 1)
	{
		volt = 10;
		RunMotor(volt, clockwise);
		AlSet_Flag = 0;
		SetHome_Flag = 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Encoder_X_Pin && AlSet_Flag == 0)
	{
		volt = 0;
		RunMotor(volt, clockwise);
		angle_sum = 0;
		AlSet_Flag = 1;
		ArmState = FinishSetHome;
	}

	if (GPIO_Pin == Emergency_Pin)
	{
		if (HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_SET)
		{
			Emergency_status = 1;
			HAL_GPIO_WritePin(PilotLamp_GPIO_Port, PilotLamp_Pin, GPIO_PIN_SET);
		}
		else
		{
			Emergency_status = 0;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if (htim == &htim4 && (Go_Flag || GenVolt_Flag) && !SetHome_Flag && AlSet_Flag)
	if (htim == &htim4 && Go_Flag && ArmState == RunPID)
	{
		//genvol
		Unwrap();
		BackwardDifference();
		TrajectoryEvaluation();
		kalmanfilter();
		volt = Cascade(theta, position_kalman, omega, omega_kalman);
		RunMotor(volt, PID_dir);
		t+=dt;
		trigger = 1;
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

	//data_input[0] = angle_sum;
	data_input[0] = omega_tosensor;

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
	position_kalman = angle_sum;
	omega_kalman = data_x_new[1];
	alpha_kalman = data_x_new[2];
	jerk_kalman = data_x_new[3];
}

void TrajectoryGenerator()
{	if(TrajectoryGenerator_Flag)
	{
		//Variable For Consider Case
		static uint8_t M;
		static uint8_t N;
		static float Va;
		static float Sa;
		static float Sv;

		theta_0 = angle_sum;

		theta_dest = theta_f - theta_0;

		if(w_max*j_max < pow(a_max,2)){
			M=1;
			N=0;
		}
		else if(w_max*j_max >= pow(a_max,2)){
			M=0;
			N=1;
		}

		Va = pow(a_max,2)/j_max;
		Sa = (2*pow(a_max,3))/(pow(j_max,2));
		Sv = w_max*(M*(2*sqrt(w_max/j_max))+N*((w_max/a_max)+(a_max/j_max)));

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
					tj = pow((theta_dest/(2*j_max)),0.33);
					ta = tj;
					tv = 2*tj;
				}
				else if(theta_dest > Sv){
					//caseIII
					tj = sqrt((w_max/j_max));
					ta = tj;
					tv = theta_dest/w_max;
				}
			}
		}
		else if(w_max > Va){
			if(theta_dest < Sa){
				//caseII
				tj = pow((theta_dest/(2*j_max)),0.33);
				ta = tj;
				tv = 2*tj;
			}
			else if(theta_dest > Sa){
				if(theta_dest < Sv){
					//caseVI
					tj = a_max/j_max;
					ta = 0.5*(sqrt(((4*theta_dest*pow(j_max,2))+pow(a_max,3))/(a_max*pow(j_max,2)))-(a_max/j_max));
					tv = ta + tj;
				}
				else if(theta_dest > Sv){
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

		theta = theta_0;
		omega = 0;
		alpha = 0;
	}
}

void TrajectoryEvaluation()
{
	if( 0 <= t && t < t1){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0) + j_max*pow(dt,3.0)/6.0;
		omega = omega + alpha*dt + 0.5*j_max*pow(dt,2.0);
		alpha = alpha + j_max*dt;
	}
	else if (t1 <= t && t< t2){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0);
		omega = omega + alpha*dt;
		alpha = alpha;
	}
	else if (t2 <= t && t < t3){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0) - j_max*pow(dt,3.0)/6.0;
		omega = omega + alpha*dt - 0.5*j_max*pow(dt,2.0);
		alpha = alpha - j_max*dt;
	}
	else if (t3 <= t && t < t4 ){
		theta = theta + omega*dt;
		omega = omega;
		alpha = 0;
	}
	else if (t4 <= t && t < t5 ){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0) - j_max*pow(dt,3.0)/6.0;
		omega = omega + alpha*dt - 0.5*j_max*pow(dt,2.0);
		alpha = alpha - j_max*dt;

	}
	else if (t5 <= t && t < t6 ){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0);
		omega = omega + alpha*dt;
		alpha = alpha;
	}
	else if (t6 <= t && t < t7 ){
		theta = theta + omega*dt + 0.5*alpha*pow(dt,2.0) + j_max*pow(dt,3.0)/6.0;
		omega = omega + alpha*dt + 0.5*j_max*pow(dt,2.0);
		alpha = alpha + j_max*dt;
	}
	else if (t7+2 <= t ){
		theta = theta_f;
		omega = omega;
		alpha = alpha;
		t = 0;
		Go_Flag = 0;
		ArmState = FinishPID;
	}
}

float PositionController(float r,float y) //r == trajectory, y==feedback
{
	e1 = r - y;
	s1 = s1 + e1;
	u1 = kp_1*e1 + ki_1*s1 + kd_1*(e1-p1);
	p1 = e1;
	return u1;
}

float VelocityController(float r,float y,float uP)
{
	e2 = uP + r;
	if(e2 >= w_max){
		e2 = w_max;
	}
	e2 = e2 - y;
	s2 = s2 + e2;
	u2 = kp_2*e2 + ki_2*s2;
	return u2;
}

float Cascade(float Pd,float P,float Vd,float V){
	static float u;
	float add = 2;
	u = PositionController(Pd, P);
	u = VelocityController(Vd, V, u);
	PID_dir = 1;
	if(u > 24){
		u = 24;
	}
	else if (u < 0){
		u = -(u);
		PID_dir = 0;
	}

	if(t >= t5)
	{
		add = 0;
	}

	return u+add;
}

void genvolt()
{
	static float angle = 0;
	if (GenVolt_Flag)
	{
		switch(state)
		{
		case 0:
			volt = 0;
			tt += 1;
			if (tt >= 1000){
				state = 1;
				tt = 0;
			}
			break;

		case 1:
			volt = 24;
			tt += 1;
			if (tt >= 4000){
				state = 2;
				tt = 0;
				volt = 0;
			}
			break;

		case 2:
			volt = 0;
			tt += 1;
			if (tt >= 1000){
				state = 3;
				tt = 0;
			}
			break;

		case 3:
			volt = (6.0/1000.0)*tt;
			tt += 1;
			if (tt >= 4000)
			{
				state = 4;
				tt = 0;
				volt = 0;
			}
			break;

		case 4:
			volt = 0;
			tt += 1;
			if (tt >= 1000)
			{
				state = 5;
				tt = 0;
				volt = 0;
			}
			break;

		case 5:
			volt = 12.0 + (12.0 * sin(angle));
			angle = (tt/1000)*2*M_PI;
			tt += 1;
			if (tt >= 4000)
			{
				state = 0;
				tt = 0;
				volt = 0;
				GenVolt_Flag = 0;
			}
			break;
		}
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
