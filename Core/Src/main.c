/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "myDef.h"
#include <stdlib.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch3;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Re-maps a number from one range to another.
 * @param x The number to be mapped.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @return The mapped value.
 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Re-maps a floating-point number from one range to another.
 * @param x The number to be mapped.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @return The mapped floating-point value.
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float ComputePIDadpt(volatile PID_t *pid, const float measuredValue, const float dt) {
	// Calculate error
	float error = pid->setPoint - measuredValue;

	// Deadband (Optional: Prevents small oscillations)
	if (fabsf(error) < pid->deadband) {
		error = 0.0f;
	}

	// Proportional term
	float Pout = pid->Kp * error;

	// Integral term (PI and PID modes)
	float Iout = 0.0f;
	if ((pid->Ki > 0.0f)) {
		pid->integral += error * dt;
		Iout = pid->Ki * pid->integral;

	}

	// Derivative term (PD and PID modes)
	float Dout = 0.0f;
	if ((pid->Kd > 0.0f)) {
		float derivative = (error - pid->prevError) / dt;
		Dout = pid->Kd * derivative;
	}

	// Compute total output
	float output = Pout + Iout + Dout;

	// Apply output limits
	if (output > pid->maxOutput - pid->minOutput) {
		pid->integral = pid->lastIntegral;
		output = pid->maxOutput - pid->minOutput;
	} else {
		pid->lastIntegral = pid->integral;
	}
	if (output < 0)
		output = 0;

	// Store values for next iteration
	pid->prevError = error;

	return output + pid->minOutput;
}

void mootorSpeed(int16_t _speed) {
	if (_speed > motorSpeedMax) {
		_speed = motorSpeedMax;
	}
	if (_speed < motorSpeedMin) {
		_speed = motorSpeedMin;
	}

	if (_speed > 0) {
		motorMode = mMode_For;
		Speed = _speed;

	} else if (_speed < 0) {
		motorMode = mMode_back;
		Speed = _speed;

	} else if (_speed == 0) {
//		if (mSVelocity > brakeThrshld) {
//			motorMode = mMode_breaking;
//			breakingPower = _speed;
//			Speed = 0;
//		} else if (mSVelocity < brakeThrshldLow) {
			Speed = 0;
			motorMode = mMode_Stop;
//		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim4) { //timer 100hz
		uint32_t currCNT = TIM3->CNT;
		int32_t delta = (int32_t) (currCNT - lastCNT); // menghindari overflow/underflow
		lastCNT = currCNT;
		dump1 = (float) delta / -28.0f;
		rps = dump1 / 0.01f;
		rpmMotor = rps * 60.0f;
		rpsWheel = rps / 10.28f;
		mmSVelocity = rpsWheel * 329.86722f;
		mSVelocity = mmSVelocity / 1000.0f;

		if (PID.setPoint == 0.0f) {
			PID.integral = 0.0f;
			PID.prevError = 0.0f;
			PID.lastIntegral = 0.0f;
			mootorSpeed(0);
		} else {

			if (receivedDataControl.dummy) {
				mootorSpeed(-(int16_t)ComputePIDadpt(&PID, -mSVelocity, 0.01f));
			} else {
				mootorSpeed((int16_t) ComputePIDadpt(&PID, mSVelocity, 0.01f));
			}

//			mootorSpeed((int16_t)ComputePIDadpt(&PID, mSVelocity, 0.01f));
		}
	}
}

void motorLOOP(void) {

//	if (Speed == 0) {
//		motorMode = zeroMode;
//	} else
//	if(Speed > 0){
//		motorMode = mMode_For;
//	}else if (Speed < 0) {
//		motorMode = mMode_back;
//	} else{
//		motorMode = mMode_Stop;
//	}


	switch (motorMode) {
	case mMode_Stop:
		val_pwmRED = 0;
		val_pwmBLACK = 0;

		clrIO(enBLACK_GPIO_Port,enBLACK_Pin);
		clrIO(enRED_GPIO_Port,enRED_Pin);
		break;
	case mMode_For:
		val_pwmRED = abs(Speed);
		val_pwmBLACK = 0;

		setIO(enBLACK_GPIO_Port, enBLACK_Pin);
		setIO(enRED_GPIO_Port, enRED_Pin);
		break;
	case mMode_back:
		val_pwmRED = 0;
		val_pwmBLACK = abs(Speed);

		setIO(enBLACK_GPIO_Port, enBLACK_Pin);
		setIO(enRED_GPIO_Port, enRED_Pin);
		break;
	case mMode_deAcsFor2stop:

		break;
	case mMode_deAcsBack2stop:

		break;
	case mMode_breaking:
		val_pwmRED = breakingPower;
		val_pwmBLACK = breakingPower;

		setIO(enBLACK_GPIO_Port, enBLACK_Pin);
		setIO(enRED_GPIO_Port, enRED_Pin);
		break;
	}
}

uint8_t chechOverlapIMU(uint8_t data){
	static uint8_t mode = 0;

	switch (mode) {
	case 0:
		if (data == 0x55) {
			mode = 1;
		}
		break;
	case 1:
		switch (data) {
		case 0x51:
			mode = 0;
			return 1;
			break;
		case 0x52:
			mode = 0;
			return 2;
			break;
		case 0x53:
			mode = 0;
			return 3;
			break;
		}

		break;
	}
	mode = 0;
	return 0;

}

void ADXL_Read(uint8_t reg, uint8_t *data, uint8_t size){

//	HAL_I2C_Mem_Read(&hi2c2, ADXL345_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size,HAL_MAX_DELAY);
}

void ADXL_Write(uint8_t reg, uint8_t *data){
//	HAL_I2C_Mem_Write(&hi2c2, ADXL345_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void ADXL_setRegisterBit(uint8_t regAdress, uint8_t bitPos, uint8_t state) {
	uint8_t _b = 0;
//    readFrom(regAdress, 1, &_b);
    ADXL_Read(regAdress, &_b, 1);
    if (state) {
        _b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
    } else {
        _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
    }
    ADXL_Write(regAdress, &_b);
//    writeTo(regAdress, _b);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

//  uint8_t devID = 0;
//  	do{
//  		ADXL_Read(DEVID_REG, &devID, 1);
//  		HAL_Delay(10);
//  	}while(devID != 0xE5);

	for (int i = 0; i < 5; ++i) {
		setIO(ledA_GPIO_Port, ledA_Pin);
		HAL_Delay(50);
		clrIO(ledA_GPIO_Port, ledA_Pin);
		HAL_Delay(50);

		setIO(ledB_GPIO_Port, ledB_Pin);
		HAL_Delay(50);
		clrIO(ledB_GPIO_Port, ledB_Pin);
		HAL_Delay(50);

		setIO(ledC_GPIO_Port, ledC_Pin);
		HAL_Delay(50);
		clrIO(ledC_GPIO_Port, ledC_Pin);
		HAL_Delay(50);
	}

//	ADXL_Write(POWER_CTL_REG, (uint8_t*)0b00000000); // StandBy Mode
//	HAL_Delay(50);
////
////	ADXL_Write(BW_RATE_REG, (uint8_t*)0b00001011); // bandwidht 100hz
//	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00001000); // Full Resolution
//	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00000011); // 16g, 10bit Data acording to range
////	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00001011); // All
//	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b10000000); // Self test
//	HAL_Delay(50);
//
////	ADXL_Write(INT_ENABLE_REG, (uint8_t*)0b10000000); // Turn On DATA_READY
////	ADXL_Write(FIFO_CTL_REG, (uint8_t*)0b01000000); // Turn On FIFO Mode
//
////	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00000000); // 4g
////	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00000001); // Full Resolution
////	ADXL_Write(DATA_FORMAT_REG, (uint8_t*)0b00001010); // All
////	HAL_Delay(50);
//
//	ADXL_Write(INT_ENABLE_REG, (uint8_t*)0b10000000); // Turn On DATA_READY
//	HAL_Delay(50);
//
//	ADXL_Write(POWER_CTL_REG, (uint8_t*)0b00001000); // Mesuring Mode
//	HAL_Delay(50);


  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, &val_pwmRED, 1);
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, &val_pwmBLACK, 1);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, &val_pwmSERVO, 1);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, valADC, totalADC);

//  HAL_UArt_
//  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//  LL_
  HAL_UART_Receive_DMA(&huart1, rx_DMA_buff_LoRa, 6);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


//  HAL_UART_Receive_DMA(&huart3, rx_DMA_buff_IMU, 15);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
		if (sendTXbuff) {
			sendTXbuff = 0;
//			HAL_UART_Transmit(&huart3, tx_buffIMU, 3, 100);
		}

		uint32_t currTime = HAL_GetTick();
		if (currTime - kHzLoop >1) {


//			mootorSpeed(speedTest);
			motorLOOP();



			if (currTime - rxTime > rxTimeout) {
//				HAL_TIM_StateTypeDef temp =HAL_TIM_PWM_GetState(&htim2);
//				HAL_UART_AbortReceive_IT(&huart1);
//				HAL_Delay(50);
//				HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
				retrying++;
//				HAL_UARTEx_ReceiveToIdle_IT(huart, pData, Size)
//				if (temp == HAL_TIM_STATE_BUSY) {
//					HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
//				}
			} else {
//				if (HAL_TIM_PWM_GetState(&htim2) == HAL_TIM_STATE_READY) {
//					HAL_UART_Receive_DMA(&huart3, rx_buff_IMU, dmaBufSize);
//				}
			}
		}

		if (currTime - slowLoop > 100) {

//			uint8_t x = 0;
//			ADXL_Read(INT_SOURCE_REG, &x, 1);
//			if (x & 128) {
//				if (!readIO(ledC_GPIO_Port, ledC_Pin))
//					setIO(ledC_GPIO_Port, ledC_Pin);
//				else
//					clrIO(ledC_GPIO_Port, ledC_Pin);
//			}else{
//				setIO(ledC_GPIO_Port, ledC_Pin);
//			}

//			uint8_t data[6];
//			memset(data, 0, sizeof(data));
//			ADXL_Read(DATA_REG, data, 6);
//			if (data[5]) {
//				if (!readIO(ledB_GPIO_Port, ledB_Pin))
//					setIO(ledB_GPIO_Port, ledB_Pin);
//				else
//					clrIO(ledB_GPIO_Port, ledB_Pin);
//
//				ADXL_Data.acc_X = (int16_t) ((data[1] << 8) | data[0]);
//				ADXL_Data.acc_Y = (int16_t) ((data[3] << 8) | data[2]);
//				ADXL_Data.acc_Z = (int16_t) ((data[5] << 8) | data[4]);
//
//				// Convert raw data to acceleration in m/s²
//				float ax = ADXL_Data.acc_X * SCALE_FACTOR;
//				float ay = ADXL_Data.acc_Y * SCALE_FACTOR;
//
//				// Combine acceleration vectors (X and Y only)
//				float a_total = sqrtf(ax * ax + ay * ay); // Magnitude of acceleration
//
//				// Integrate acceleration to get velocity (v = v + a * dt)
//				velocity_mps += a_total * DT;
//
//				// Convert to km/h
//				velocity_kmh = velocity_mps * 3.6f;
//			}

			slowLoop = currTime;

			//________________ ADC CAOMPUTING __________________________________
			ADCvalue.Vref = (float) ((V_REF_INT * 4095.0) / valADC[0]);
			ADCvalue.Vtemp = (float) (valADC[1] * ADCvalue.Vref) / 4095.0;
			ADCvalue.temp = (((V_AT_25C - ADCvalue.Vtemp) * 1000.0)
					/ AVG_SLOPE) + 25.0;
			ADCvalue.Vamp = (float) (valADC[2] * ADCvalue.Vref) / 4095.0;
			ADCvalue.V5amp = ADCvalue.Vamp
					* ((10000.0 + 20000.0) / 20000.0);
			ADCvalue.amp = (ADCvalue.V5amp - 2.5) / V_A;

			//_________________________________________________________________
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
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
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, enRED_Pin|enBLACK_Pin|ledA_Pin|ledB_Pin
                          |ledC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADXL_SCL_Pin ADXL_SDA_Pin */
  GPIO_InitStruct.Pin = ADXL_SCL_Pin|ADXL_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : enRED_Pin enBLACK_Pin */
  GPIO_InitStruct.Pin = enRED_Pin|enBLACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ledA_Pin ledB_Pin ledC_Pin */
  GPIO_InitStruct.Pin = ledA_Pin|ledB_Pin|ledC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void moveDataToStructPID(uint8_t *rx_buff_LoRa, sendPID_t *send_data) {
	send_data->index = rx_buff_LoRa[0];
	send_data->identifier = rx_buff_LoRa[1];

	send_data->_stearing = ((uint16_t) (rx_buff_LoRa[3] & 0xF) << 8)
			| rx_buff_LoRa[2];
	send_data->_setPoint = ((uint16_t) rx_buff_LoRa[4] << 4)
			| (rx_buff_LoRa[3] >> 4);
	send_data->_propotional = ((uint16_t) (rx_buff_LoRa[6] & 0xF) << 8)
			| rx_buff_LoRa[5];
	send_data->_integral = ((uint16_t) rx_buff_LoRa[7] << 4)
			| (rx_buff_LoRa[6] >> 4);
	send_data->_derivative = ((uint16_t) (rx_buff_LoRa[9] & 0xF) << 8)
			| rx_buff_LoRa[8];
	send_data->checkSum = ((uint16_t) rx_buff_LoRa[10] << 4)
			| (rx_buff_LoRa[9] >> 4);
}

void moveDataToStructControl(uint8_t *rx_buff_LoRa, sendControl_t *send_data) {
	send_data->index = rx_buff_LoRa[0];
	send_data->indetifier = rx_buff_LoRa[1];

	send_data->_stearing = ((uint16_t) (rx_buff_LoRa[3] & 0xF) << 8)
			| rx_buff_LoRa[2];
	send_data->_setPoint = ((uint16_t) rx_buff_LoRa[4] << 4)
			| (rx_buff_LoRa[3] >> 4);
	send_data->dummy = ((uint16_t) rx_buff_LoRa[5] & 0xF);
	send_data->checkSum = ((uint16_t) rx_buff_LoRa[6] << 4)
			| (rx_buff_LoRa[5] >> 4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	HAL_UART_Receive_DMA(&huart1, rx_buff_IMU, 11); //You need to toggle a breakpoint on this line!
	if (huart == &huart1) {
		uint32_t currTime = HAL_GetTick();
//		if (!readIO(ledA_GPIO_Port, ledA_Pin))
//			setIO(ledA_GPIO_Port, ledA_Pin);
//		else
//			clrIO(ledA_GPIO_Port, ledA_Pin);

//		if (rx_buff_LoRa[0] == signiture1) {
//			if (rx_buff_LoRa[1] == 0x36) {
//				if (!readIO(ledA_GPIO_Port, ledA_Pin))
//					setIO(ledA_GPIO_Port, ledA_Pin);
//				else
//					clrIO(ledA_GPIO_Port, ledA_Pin);
//			}
//		}

		for (int i = 0; i < 6; ++i) {
			switch (captureMode) {
			case 0:
				if (rx_DMA_buff_LoRa[i] == signiture1) {
					rx_buff_LoRa[0] = rx_DMA_buff_LoRa[i];
					captureMode = 1;
				}
				break;
			case 1:
				if (rx_DMA_buff_LoRa[i] == signiture2) {
					rx_buff_LoRa[1] = rx_DMA_buff_LoRa[i];
					captureMode = 2;
					currIndex = 2;
				} else if (rx_DMA_buff_LoRa[i] == 0x36) {
					rx_buff_LoRa[1] = rx_DMA_buff_LoRa[i];
					captureMode = 3;
					currIndex = 2;
				} else {
					captureMode = 1;
				}
				break;
			case 2:
				rx_buff_LoRa[currIndex] = rx_DMA_buff_LoRa[i];
				// update time saat sudah menerima data
				rxTime = currTime;
				currIndex++;

				if (currIndex > 10) {
					currIndex = 2;
					moveDataToStructPID(rx_buff_LoRa, &receivedDataPID);

					uint16_t checkSum = 0;
					checkSum += receivedDataPID.index;
					checkSum += receivedDataPID.identifier;
					checkSum += receivedDataPID._stearing;
					checkSum += receivedDataPID._setPoint;
					checkSum += receivedDataPID._derivative;
					checkSum += receivedDataPID._integral;
					checkSum += receivedDataPID._propotional;

					//				val_pwmSERVO = map(receivedDataPID._stearing, 50, 4045, leftPos, rightPos);

					checkSum &= 0xFFF;
					if (receivedDataPID.checkSum == checkSum) {
						correctData++;

						PID.Kp = mapf((float)receivedDataPID._propotional, 0.0, 4095.0, 0.0, 2000.0);
						PID.Ki = mapf((float)receivedDataPID._integral, 0.0, 4095.0, 0.0, 2000.0);
						PID.Kd = mapf((float)receivedDataPID._derivative, 0.0, 4095.0, 0.0, 2000.0);
					}
					captureMode = 1;
				}
				break;
			case 3:
				rx_buff_LoRa[currIndex] = rx_DMA_buff_LoRa[i];
				// update time saat sudah menerima data
				rxTime = currTime;
				currIndex++;

				if (currIndex > 6) {
					currIndex = 2;
					moveDataToStructControl(rx_buff_LoRa, &receivedDataControl);

					uint16_t checkSum = 0;
					checkSum += receivedDataControl.index;
					checkSum += receivedDataControl.indetifier;
					checkSum += receivedDataControl._stearing;
					checkSum += receivedDataControl._setPoint;
					checkSum += receivedDataControl.dummy;

					checkSum &= 0xFFF;
					if (receivedDataControl.checkSum == checkSum) {
						correctData++;

						if (!readIO(ledA_GPIO_Port, ledA_Pin))
							setIO(ledA_GPIO_Port, ledA_Pin);
						else
							clrIO(ledA_GPIO_Port, ledA_Pin);

						val_pwmSERVO = map(receivedDataControl._stearing, 500,
								4095 - 500, leftPos, rightPos);
						PID.setPoint = mapf((float)receivedDataControl._setPoint, 0.0, 4095.0, 0.0, 4.0);

//						if (receivedDataControl.dummy) {
//							val_pwmRED = 0;
//							val_pwmBLACK = map(receivedDataControl._setPoint, 0,
//									4095, 0, 2000);
//							setIO(enBLACK_GPIO_Port, enBLACK_Pin);
//							setIO(enRED_GPIO_Port, enRED_Pin);
//						} else {
//							val_pwmRED = map(receivedDataControl._setPoint, 0,
//									4095, 0, 2000);
//							val_pwmBLACK = 0;
//							setIO(enBLACK_GPIO_Port, enBLACK_Pin);
//							setIO(enRED_GPIO_Port, enRED_Pin);
//						}
//
//						if (receivedDataControl._setPoint == 0) {
//							clrIO(enBLACK_GPIO_Port, enBLACK_Pin);
//							clrIO(enRED_GPIO_Port, enRED_Pin);
//						}

						controlInterval = currTime - lastControlTime;
						lastControlTime = currTime;
					}
					captureMode = 1;
				}
				break;
			}
		}




//		switch (captureMode) {
//		case 0:
//			if (rx_byte == signiture1) {
//				rx_buff_LoRa[0] = rx_byte;
//				captureMode = 1;
//			}
//			break;
//		case 1:
//			if (rx_byte == signiture2) {
//				rx_buff_LoRa[1] = rx_byte;
//				captureMode = 2;
//				currIndex = 2;
//			} else if (rx_byte == 0x36) {
//				rx_buff_LoRa[1] = rx_byte;
//				captureMode = 3;
//				currIndex = 2;
//			} else {
//				captureMode = 1;
//			}
//			break;
//		case 2:
//			rx_buff_LoRa[currIndex] = rx_byte;
//			// update time saat sudah menerima data
//			rxTime = currTime;
//			currIndex++;
//
//			if (currIndex > 10) {
//				currIndex = 2;
//				moveDataToStructPID(rx_buff_LoRa, &receivedDataPID);
//
//				uint16_t checkSum = 0;
//				checkSum += receivedDataPID.index;
//				checkSum += receivedDataPID.identifier;
//				checkSum += receivedDataPID._stearing;
//				checkSum += receivedDataPID._setPoint;
//				checkSum += receivedDataPID._derivative;
//				checkSum += receivedDataPID._integral;
//				checkSum += receivedDataPID._propotional;
//
////				val_pwmSERVO = map(receivedDataPID._stearing, 50, 4045, leftPos, rightPos);
//
//				checkSum &= 0xFFF;
//				if (receivedDataPID.checkSum == checkSum) {
//					correctData++;
//				}
//				captureMode = 1;
//			}
//			break;
//		case 3:
//			rx_buff_LoRa[currIndex] = rx_byte;
//			// update time saat sudah menerima data
//			rxTime = currTime;
//			currIndex++;
//
//			if (currIndex > 6) {
//				currIndex = 2;
//				moveDataToStructControl(rx_buff_LoRa, &receivedDataControl);
//
//				uint16_t checkSum = 0;
//				checkSum += receivedDataControl.index;
//				checkSum += receivedDataControl.indetifier;
//				checkSum += receivedDataControl._stearing;
//				checkSum += receivedDataControl._setPoint;
//				checkSum += receivedDataControl.dummy;
//
//				val_pwmSERVO = map(receivedDataControl._stearing, 500, 4095-500, leftPos, rightPos);
//
//				checkSum &= 0xFFF;
//				if (receivedDataControl.checkSum == checkSum) {
//					correctData++;
//				}
//				captureMode = 1;
//			}
//			break;
//		}
//
//		if (!stopReceive) {
//			curreErrorUARTRceive = HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
////			UART
//			if (curreErrorUARTRceive != HAL_OK) {
//				stopReceive = 1;
//			}
//		}
	}

	if (0) {
//		uint32_t currTime = HAL_GetTick();
//		memcpy(&rx_main_IMU[])

//		if (rx_buff_IMU[0] != 0x55 || !sync) {
//			static uint8_t headerIndex = 0;
//			for (int i = 0; i < 11; ++i) {
//				if (rx_buff_IMU[i] == 0x55) {
//					headerIndex = i;
//					sync = 0;
//
//					break;
//				}
//			}
//		}else{
//			//parse
//		}
//		break;

		for (int i = 0; i < 15; ++i) {
			switch (captureModeIMU) {
			case 0:
				if (rx_DMA_buff_IMU[i] == 0x55) {
					rx_buff_IMU[0] = rx_DMA_buff_IMU[i];
					captureModeIMU = 1;
				}
				break;
			case 1:
				if (rx_DMA_buff_IMU[i] == 0x51) {
					rx_buff_IMU[1] = rx_DMA_buff_IMU[i];
					captureModeIMU = 2;
					currIndexIMU = 2;
				} else if (rx_DMA_buff_IMU[i] == 0x52) {
					rx_buff_IMU[1] = rx_DMA_buff_IMU[i];
					captureModeIMU = 3;
					currIndexIMU = 2;
				} else if (rx_DMA_buff_IMU[i] == 0x53) {
					rx_buff_IMU[1] = rx_DMA_buff_IMU[i];
					captureModeIMU = 4;
					currIndexIMU = 2;
				} else {
					captureModeIMU = 1;
				}
				break;
			case 2:
				rx_buff_IMU[currIndexIMU] = rx_DMA_buff_IMU[i];
				currIndexIMU++;

				switch (chechOverlapIMU(rx_DMA_buff_IMU[i])) {
				case 1:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x51;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 2;
					break;
				case 2:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x52;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 3;
					break;
				case 3:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x53;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 4;
					break;
				}

				if (currIndexIMU > 11) {
					currIndexIMU = 2;

					uint16_t sum = 0;
					for (int x = 0; x < 10; ++x) {
						sum+=rx_buff_IMU[x];
					}

					if ((uint8_t) (sum & 0xFF) == rx_buff_IMU[10]) {
						AccelerationCount++;
						int16_t ax_raw = (rx_buff_IMU[3] << 8) | rx_buff_IMU[2];
						int16_t ay_raw = (rx_buff_IMU[5] << 8) | rx_buff_IMU[4];
						int16_t az_raw = (rx_buff_IMU[7] << 8) | rx_buff_IMU[6];
						int16_t temp_raw = (rx_buff_IMU[9] << 8)
								| rx_buff_IMU[8];

						acc.acc_x_g = (float) ax_raw / 32768.0f * 16.0f;
						acc.acc_y_g = (float) ay_raw / 32768.0f * 16.0f;
						acc.acc_z_g = (float) az_raw / 32768.0f * 16.0f;
						acc.acc_total_g = sqrt(
								(acc.acc_x_g * acc.acc_x_g)
										+ (acc.acc_y_g * acc.acc_y_g)
//										+ (acc.acc_z_g * acc.acc_z_g)
										);
//						acc.acc_total_g -= 1.0f;
						acc.acc_total_ms = acc.acc_total_g * Gs;
						acc.temperature_c = ((float) temp_raw / 32768.0f)
								* 96.36f + 36.53f;

						if (!readIO(ledB_GPIO_Port, ledB_Pin))
									setIO(ledB_GPIO_Port, ledB_Pin);
								else
									clrIO(ledB_GPIO_Port, ledB_Pin);
				}

					captureModeIMU = 1;
				}
				break;
			case 3:
				rx_buff_IMU[currIndexIMU] = rx_DMA_buff_IMU[i];
				currIndexIMU++;

				switch (chechOverlapIMU(rx_DMA_buff_IMU[i])) {
				case 1:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x51;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 2;
					break;
				case 2:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x52;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 3;
					break;
				case 3:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x53;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 4;
					break;
				}

				if (currIndexIMU > 11) {
					currIndexIMU = 2;

					captureModeIMU = 1;
				}
				break;
			case 4:
				rx_buff_IMU[currIndexIMU] = rx_DMA_buff_IMU[i];
				currIndexIMU++;

				switch (chechOverlapIMU(rx_DMA_buff_IMU[i])) {
				case 1:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x51;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 2;
					break;
				case 2:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x52;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 3;
					break;
				case 3:
					rx_buff_IMU[0] = 0x55;
					rx_buff_IMU[1] = 0x53;
					currIndexIMU = 2;
					reLocation++;
					captureModeIMU = 4;
					break;
				}

				if (currIndexIMU > 11) {
					currIndexIMU = 2;

					captureModeIMU = 1;
				}
				break;
			}
		}

//		uint16_t sum = 0;
//		for (int8_t i = 0; i < 10; i++) {
//			sum += rx_buff_IMU[i];
//		}
//
//		if ((uint8_t) (sum & 0xFF) == rx_buff_IMU[10]) {
//			if (!readIO(ledB_GPIO_Port, ledB_Pin))
//				setIO(ledB_GPIO_Port, ledB_Pin);
//			else
//				clrIO(ledB_GPIO_Port, ledB_Pin);
//
//			if (rx_buff_IMU[0] == 0x55) {
//				switch (rx_buff_IMU[1]) {
//				case 0x51:
//					AccelerationCount++;
//
//					int16_t ax_raw = (rx_buff_IMU[3] << 8) | rx_buff_IMU[2];
//					int16_t ay_raw = (rx_buff_IMU[5] << 8) | rx_buff_IMU[4];
//					int16_t az_raw = (rx_buff_IMU[7] << 8) | rx_buff_IMU[6];
//					int16_t temp_raw = (rx_buff_IMU[9] << 8) | rx_buff_IMU[8];
//
//					acc.acc_x_g = (float)ax_raw / 32768.0f * 16.0f;
//					acc.acc_y_g = (float)ay_raw / 32768.0f * 16.0f;
//					acc.acc_z_g = (float)az_raw / 32768.0f * 16.0f;
////					acc.acc_z_g = ((float)az_raw / 32768.0f * 16.0f)-1.0f;
//					acc.acc_total_g = sqrt((acc.acc_x_g * acc.acc_x_g) + (acc.acc_y_g * acc.acc_y_g) + (acc.acc_z_g * acc.acc_z_g));
//					acc.acc_total_ms = acc.acc_total_g * Gs;
//					acc.temperature_c = ((float)temp_raw / 32768.0f) * 96.36f + 36.53f;
//					break;
//				case 0x52:
//					AngularVelocityCount++;
//
//					int16_t gx_raw = (rx_buff_IMU[3] << 8) | rx_buff_IMU[2];
//					int16_t gy_raw = (rx_buff_IMU[5] << 8) | rx_buff_IMU[4];
//					int16_t gz_raw = (rx_buff_IMU[7] << 8) | rx_buff_IMU[6];
//					int16_t volt_raw = (rx_buff_IMU[9] << 8) | rx_buff_IMU[8];
//
//					gyro.gyro_x_dps = (float)gx_raw / 32768.0f * 2000.0f;
//					gyro.gyro_y_dps = (float)gy_raw / 32768.0f * 2000.0f;
//					gyro.gyro_z_dps = (float)gz_raw / 32768.0f * 2000.0f;
//					gyro.voltage_v = (float)volt_raw / 100.0f;
//					break;
//				case 0x53:
//					AngleCount++;
//
//					int16_t roll_raw = (rx_buff_IMU[3] << 8) | rx_buff_IMU[2];
//					int16_t pitch_raw = (rx_buff_IMU[5] << 8) | rx_buff_IMU[4];
//					int16_t yaw_raw = (rx_buff_IMU[7] << 8) | rx_buff_IMU[6];
//					uint16_t version_raw = (rx_buff_IMU[9] << 8) | rx_buff_IMU[8];
//
//					angle.roll_deg = (float)roll_raw / 32768.0f * 180.0f;
//					angle.pitch_deg = (float)pitch_raw / 32768.0f * 180.0f;
//					angle.yaw_deg = (float)yaw_raw / 32768.0f * 180.0f;
//					angle.version = version_raw;
//					break;
//				}
//			}
//		}
	}
//	HAL_UART_Receive_DMA(&huart1, rx_buff_IMU, 11); //You need to toggle a breakpoint on this line!
}

//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
//	if (huart == &huart3) {
//		haltCallUART3++;
//	}
//}
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
