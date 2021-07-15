/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *stream)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Buffer Maximum Size
#define MAX_BUFF 32

// Reference position counter value
#define HOME_CNT 32767
#define RXBUFSIZE 64
#define STR_SIZE 7
#define BUFF_SIZE 16
#define N_STRING 10
#define NUM 5
#define N_ORDERS 14
#define STR_SIZE 7
#define MIN_DUTY 0
#define HALF_DUTY 2047
#define MAX_DUTY 4095
#define SECTOR3_ADDR 0x0800C000
#define MEMBUFF 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

char orders[N_ORDERS][STR_SIZE] = { "AT+MOVE", "AT+VELO", "AT+LOAD", "AT+SAVE",
		"AT+LIST", "AT+CLRM", "AT+RSET", "AT+CNTL", "AT+ENCO", "AT+HOME",
		"AT+OPEN", "AT+CLSE", "AT+STRT", "AT+STOP" };
char split[N_STRING][BUFF_SIZE];

char buffer[7];
uint8_t num;
uint32_t encAct = 0, encPrev;
uint8_t rxFilled = 0;
uint8_t rxCnt = 0;
uint8_t rxBufPos = 0;
uint8_t rxBuf[RXBUFSIZE];
uint8_t N, dmax;
uint16_t start, len, aux;
uint32_t moveVars[4];
int aux2;
char message[RXBUFSIZE];
uint32_t enco[6], velo[10] = { HALF_DUTY,
HALF_DUTY,
HALF_DUTY, HALF_DUTY, HALF_DUTY, HALF_DUTY, HALF_DUTY, HALF_DUTY,
HALF_DUTY, HALF_DUTY };
uint32_t position[6] = { HOME_CNT, HOME_CNT, HOME_CNT, HOME_CNT, HOME_CNT,
HOME_CNT };
uint32_t encoActual[6];
int orderNums[7];
long int error[6], offset[6] = { 0, 0, 0, 0, 0, 0 };
char split[N_STRING][BUFF_SIZE];
uint8_t uartRX[32];
uint8_t pressed;
float32_t pwm[6];
uint8_t homeON = 128;
uint32_t homeStage_0[4] = { 65000, 15000, 40000, 40000 }, homeStage_1[4] = {
		5000, 35000, 10000, 20000 };
uint8_t stage[4];
uint32_t initSw[5], finSw[5];
uint32_t encTime[4] = { 250000, 50000, 120000, 150000 };
uint32_t sameEnc, temp;
uint8_t mode = 0;
uint8_t gameSaveLoad = 0;
uint32_t modeCNT = 0;
uint8_t gripStatus = 0;
uint32_t cnt;
uint32_t x, y;
extern uint8_t cancelWord[4];
extern uint8_t cleanWord[4]; // ____
extern uint8_t cntlWord[4]; // cntL
extern uint8_t doneWord[4]; // donE
extern uint8_t homeWord[4];	// Hone
extern uint8_t loadWord[4];	// LoAd
extern uint8_t saveWord[4];	// SAvE
extern uint8_t startWord[4]; // Strt
extern uint8_t stopWord[4]; // stoP
extern uint8_t zeroWord[4]; // Digit 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM12_Init();
	MX_TIM13_Init();
	MX_TIM14_Init();
	MX_ADC1_Init();
	MX_SPI3_Init();
	/* USER CODE BEGIN 2 */

	// Start Timers
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	// Encoder 1
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// Encoder 2
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	// Encoder 3
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	// Encoder 4
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);	// Encoder 5
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);	// Encoder 6
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);		// Motor 1
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);		// Motor 2
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);		// Motor 3
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);		// Motor 4
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);		// Motor 5
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);		// Motor 6

	// Start ADC
	HAL_ADC_Start_DMA(&hadc1, moveVars, sizeof(moveVars) >> 2);

	// Start UART2
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, rxBuf, RXBUFSIZE);

	// Initialize Led Driver
	initMatrix();
	sendWord(cleanWord);
	// Initialize PIDs
	// Base
	arm_pid_instance_f32 PID0;
	PID0.Kp = 8;			//PID_PARAM_KP;
	PID0.Ki = 0;			//PID_PARAM_KI;
	PID0.Kd = 0.5;			//PID_PARAM_KD;
	arm_pid_init_f32(&PID0, 1);

	// Shoulder
	arm_pid_instance_f32 PID1;
	PID1.Kp = 32.27;			//PID_PARAM_KP;
	PID1.Ki = 0;			//PID_PARAM_KI;
	PID1.Kd = 1;			//PID_PARAM_KD;
	arm_pid_init_f32(&PID1, 1);

	// Elbow
	arm_pid_instance_f32 PID2;
	PID2.Kp = 14;			//PID_PARAM_KP;
	PID2.Ki = 0;			//PID_PARAM_KI;
	PID2.Kd = 0.5;			//PID_PARAM_KD;
	arm_pid_init_f32(&PID2, 1);

	// Wrist
	arm_pid_instance_f32 PID3;
	PID3.Kp = 20;			//PID_PARAM_KP;
	PID3.Ki = 0;			//PID_PARAM_KI;
	PID3.Kd = 0;			//PID_PARAM_KD;
	arm_pid_init_f32(&PID3, 1);

	// Grip
	arm_pid_instance_f32 PID4;
	PID4.Kp = 4.75;			//PID_PARAM_KP;
	PID4.Ki = 0;			//PID_PARAM_KI;
	PID4.Kd = 0;			//PID_PARAM_KD;
	arm_pid_init_f32(&PID4, 1);

	// PID's Array
	arm_pid_instance_f32 PIDs[6] = { PID0, PID1, PID2, PID3, PID3, PID4 };

	// Switches Array
	GPIO_TypeDef *gpios[5] = { GPIOA, GPIOB, GPIOB, GPIOB, GPIOC };
	uint16_t gpioPorts[5] = { GPIO_PIN_15, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10,
	GPIO_PIN_0 };

	// Encoder Timers Array
	TIM_HandleTypeDef encoTimers[6] =
			{ htim1, htim2, htim3, htim4, htim5, htim8 };
	TIM_HandleTypeDef pwmTimers[6] = { htim10, htim11, htim12, htim12, htim13,
			htim14 };

	// Set Initial Encoder Counters Value
	for (uint8_t i = 0; i < 6; i++)
		encoTimers[i].Instance->CNT = HOME_CNT;

	// Activate All L298 Modules
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	printf("%s\r\n", "                                                    ");
	printf("%s\r\n", "------- Welcome To Scorbot ER Vplus Terminal -------");
	printf("%s\r\n", "____________________________________________________");
	printf("%s\r\n", "Warning!!! AT+HOME order execution is recommended");
	setSpeed(velo, 65);
	TIM14->CCR1 = HALF_DUTY;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		/*
		 * Terminal Mode
		 */
		while (mode % 2 == 0) {
			//Position Control
			getPwm(PIDs);
			if (homeON == 0)
				goto home;
		}

		/*
		 * Gamepad Mode
		 */
		while (mode % 2 != 0) {
			setSpeed(velo, 100);
			// Position Control
			getPwm(PIDs);
			// Adjust Duty Cycle
			for (int i = 0; i < 5; i++) {
				if (i < 3) {
					if (joyInRange(moveVars[i]) == 1) {
						pwmTimers[i].Instance->CCR1 = moveVars[i];
						position[i] = enco[i];
					}
				} else {
					if (joyInRange(moveVars[3]) == 1) {
						pwmTimers[3].Instance->CCR2 = moveVars[3];
						pwmTimers[4].Instance->CCR1 = 4095 - moveVars[3];
						position[i] = enco[i];
					}
				}
				enco[i] = encoTimers[i].Instance->CNT;
			}
			// Open - Close Grip
			if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)) {
				gripStatus++;
				if (gripStatus > 127)
					gripStatus = 0;
				if (gripStatus % 2 == 0)
					closeGrip(&htim14, GPIOC, GPIO_PIN_13);
				else
					openGrip(&htim14, GPIOC, GPIO_PIN_13);
			}

			//If Home Button Is Pressed, Jump To Home Label
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)) {
				homeON = 0;
				sendWord(homeWord);
				HAL_Delay(1000);
				sendWord(cleanWord);
				goto home;
			}

			//Pitch Up
			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) {
				TIM12->CCR2 = 3072;
				TIM13->CCR1 = 3072;
				enco[3] = htim4.Instance->CNT;
				enco[4] = htim5.Instance->CNT;
				position[3] = htim4.Instance->CNT;
				position[4] = htim5.Instance->CNT;
			}

			//Pitch Down
			while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
				TIM12->CCR2 = 1024;
				TIM13->CCR1 = 1024;
				enco[3] = htim4.Instance->CNT;
				enco[4] = htim5.Instance->CNT;
				position[3] = htim4.Instance->CNT;
				position[4] = htim5.Instance->CNT;
			}

			// Start, Stop & Control Mode
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)) {
				cnt = 0;
				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) && cnt < 1800000)
					cnt++;
				if (cnt >= 1200000) {
					x++;
					mode = controlMode(mode);
					sendWord(cntlWord);
					HAL_Delay(1000);
					sendWord(cleanWord);
				} else if (cnt < 1200000) {
					y++;
					if (y % 2 == 0) {
						for (uint8_t i = 0; i < 5; i++) {
							enco[i] = encoTimers[i].Instance->CNT;
							position[i] = enco[i];
						}
						startRobot(GPIOA, GPIO_PIN_10);
					} else if (y % 2 != 0) {
						stopRobot(GPIOA, GPIO_PIN_10);
					}
				}
				if (x > 250)
					x = 0;
				if (y > 250)
					y = 0;
				HAL_Delay(500);
			}

			// Save Position
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
				saveMenu(position);
			}

			// Load Position
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {
				loadMenu(position);
			}

			// Home
			home: if (homeON == 0) {
				printf("%s\r\n", "HOMING...");
				preHome();
				setSpeed(velo, 65);
				for (uint8_t j = 0; j < 4; j++) {
					resetPositions(enco, position, encoTimers);
					HAL_Delay(10);
					stage[j] = 0;
					position[j] = homeStage_0[j];
					if (j == 3) {
						position[3] = 37000;
						position[4] = 37000;
					}
					sameEnc = 0;
					while (1) {
						for (uint8_t i = 0; i < 5; i++) {
							enco[i] = encoTimers[i].Instance->CNT;
							error[i] = enco[i] - position[i] + offset[i];
							pwm[i] = arm_pid_f32(&PIDs[i], error[i]);
							pwm[i] += HALF_DUTY;
							if (pwm[i] > velo[i]) {
								pwm[i] = velo[i];
							} else if (pwm[i] < velo[i + 5]) {
								pwm[i] = velo[i + 5];
							}
							if (i == 3)
								pwmTimers[i].Instance->CCR2 = pwm[i];
							else
								pwmTimers[i].Instance->CCR1 = pwm[i];
							encoActual[i] = encoTimers[i].Instance->CNT;
						}
						if (encoActual[j] == enco[j])
							sameEnc++;

						if (sameEnc > encTime[j] && stage[j] == 0) {
							HAL_Delay(10);
							position[j] = homeStage_1[j];
							if (j == 3) {
								position[3] = 23000;
								position[4] = 23000;
							}
							stage[j]++;
						} else if (!HAL_GPIO_ReadPin(gpios[j], gpioPorts[j])
								&& stage[j] == 1) {
							initSw[j] = encoTimers[j].Instance->CNT;
							HAL_Delay(10);
							while (!HAL_GPIO_ReadPin(gpios[j], gpioPorts[j])) {
								finSw[j] = encoTimers[j].Instance->CNT;
							}
							HAL_Delay(10);
							position[j] = initSw[j] + finSw[j];
							position[j] = position[j] >> 1;
							if (j == 3) {
								temp = position[3];
								position[3] = temp + 320;
								position[4] = temp - 320;
							}
							HAL_Delay(10);
							stage[j]++;
							sameEnc = 0;
						}
						if (sameEnc > 30000 && stage[j] == 2) {
							HAL_Delay(10);
							break;
						}
					}
				}
				openGrip(&htim14, GPIOC, GPIO_PIN_13);
				HAL_Delay(3000);
				closeGrip(&htim14, GPIOC, GPIO_PIN_13);
				printf("%s\r\n", "HOMING PROCESS FINISHED SUCCESSFULLY!!!");
				resetPositions(enco, position, encoTimers);
				if (mode % 2 != 0) {
					sendWord(doneWord);
					HAL_Delay(1000);
					sendWord(cleanWord);
				}
				homeON++;
			}
		}
		/* USER CODE END 3 */
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 82;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65535;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 0;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 4095;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2047;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 0;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 4095;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2047;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */
	HAL_TIM_MspPostInit(&htim11);

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 0;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 4095;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2047;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */
	HAL_TIM_MspPostInit(&htim12);

}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void) {

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 0;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 4095;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2047;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */
	HAL_TIM_MspPostInit(&htim13);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 0;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 4095;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2047;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ENAB_GRIP_GPIO_Port, ENAB_GRIP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SEGMENT_DATA_Pin | ENABLE_MOTOR_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : ENAB_GRIP_Pin */
	GPIO_InitStruct.Pin = ENAB_GRIP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ENAB_GRIP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_5_Pin */
	GPIO_InitStruct.Pin = SW_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW_5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SEGMENT_DATA_Pin */
	GPIO_InitStruct.Pin = SEGMENT_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SEGMENT_DATA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_2_Pin SW_3_Pin SW_4_Pin */
	GPIO_InitStruct.Pin = SW_2_Pin | SW_3_Pin | SW_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LOAD_Pin SAVE_Pin */
	GPIO_InitStruct.Pin = LOAD_Pin | SAVE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : HOME_Pin START_STOP_CONTROL_Pin */
	GPIO_InitStruct.Pin = HOME_Pin | START_STOP_CONTROL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ENABLE_MOTOR_Pin */
	GPIO_InitStruct.Pin = ENABLE_MOTOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ENABLE_MOTOR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PITCH_DOWN_Pin PITCH_UP_Pin */
	GPIO_InitStruct.Pin = PITCH_DOWN_Pin | PITCH_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_1_Pin */
	GPIO_InitStruct.Pin = SW_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GRIP_OPEN_CLOSE_Pin */
	GPIO_InitStruct.Pin = GRIP_OPEN_CLOSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GRIP_OPEN_CLOSE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

GETCHAR_PROTOTYPE {
	while (!(USART2->SR & (1 << 5))) {
	};
	return USART2->DR;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	uint8_t ord;
	uint16_t N;

// Check Interrupt & Clear It
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		rxCnt++;
		start = rxBufPos;
		rxBufPos = RXBUFSIZE - (uint16_t) huart->hdmarx->Instance->NDTR;
		len = RXBUFSIZE;
		if (rxFilled < 2) {
			if (rxFilled) {
				if (rxBufPos <= start)
					len = rxBufPos + RXBUFSIZE - start;
				else
					len = RXBUFSIZE + 1;
			} else
				len = rxBufPos - start;
		} else
			len = RXBUFSIZE + 2;

		if (len < RXBUFSIZE) {
			for (uint16_t i = 0; i < RXBUFSIZE; i++)
				message[i] = 0;
			for (uint16_t i = 0; i < len; i++) {
				if (start + i >= RXBUFSIZE)
					message[i] = rxBuf[start + i - RXBUFSIZE];
				else
					message[i] = rxBuf[start + i];
			}
		} else
			printf("%s\r\n", "ERROR!!!\r\nReceived Command Is Too Long.");
		rxFilled = 0;
	} else
		rxFilled++;
	if (mode % 2 == 0) {
		ord = checkOrder(message);
		if (ord < N_ORDERS && eol(message) == 0) {
			/*AT+MOVE*/
			if (ord == 0) {
				if (checkMoveSyntax(message) == 0)
					printf("%s\r\n", "Order Received!!!");
				splitStr(orderNums, message);
				for (int i = 0; i < 5; i++) {
					position[i] = orderNums[i];

				}

			} else if (ord > 0 && ord < 5) {
				if (checkOneNumSyntax(message) == 0) {
					N = getNum(message);
					if (N >= 0 && N <= 100) {
						printf("%s\r\n", "Order Received!!!");
						switch (ord) {
						/*AT+VELO*/
						case 1:
							dmax = N;
							setSpeed(velo, dmax);
							break;
							/*AT+LOAD*/
						case 2:
							load(position, N, 5);
							break;
							/*AT+SAVE*/
						case 3:
							if (N == 0)
								printf("%s\r\n",
										"This Is Home Position.\r\nYou Are Not Allowed Modify It");
							else
								save(position, N);
							break;
						case 4:
							printList(N);
							break;
						default:
							break;
						}
					} else {
						printf("%s\r\n", "ERROR!!!\r\nNumber Out Of Range.");
						printf("%s\r\n", "Insert Values Between 0 - 100.");
					}
				} else if (checkOneNumSyntax(message) != 0)
					printf("%s\r\n", "SYNTAX ERROR!!!");

			} else if (ord > 4 && ord < 14) {
				switch (ord) {
				case 5:
					clearFlash();
					break;
				case 6:
					printf("%s\r\n", "Rebooting Microcontroller. Please Wait");
					for (uint32_t i = 0; i < 7500000; i++)
						asm("NOP");
					HAL_NVIC_SystemReset();
					break;
				case 7:
					mode = controlMode(mode);
					break;
				case 8:
					showEnco(enco);
					break;
				case 9:
					homeON = 0;
					setSpeed(velo, 65);
					break;
				case 10:
					openGrip(&htim14, GPIOC, GPIO_PIN_13);
					break;
				case 11:
					closeGrip(&htim14, GPIOC, GPIO_PIN_13);
					break;
				case 12:
					startRobot(GPIOA, GPIO_PIN_10);
					break;
				case 13:
					stopRobot(GPIOA, GPIO_PIN_10);
					break;
				default:
					break;
				}
			}
		} else {
			if (eol(message) != 0)
				printf("%s\r\n", "ERROR!!!\r\nThere Is No End Of Line.");
			else if (checkOrder(message) >= N_ORDERS)
				printf("%s\r\n", "ERROR!!!\r\nThe Order Does Not Exist.");
			else if (checkSyntax(message) != 0)
				printf("%s\r\n", "SYNTAX ERROR!!!");
		}
	} else if (strncmp(message, "AT+CNTL", 7) == 0)
		mode = controlMode(mode);
	else
		printf("%s\r\n", "Terminal Mode Is Disabled!!!");
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
