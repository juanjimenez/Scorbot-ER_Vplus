#include <main.h>

#define BUFF_SIZE 16
#define N_STRING 10
#define NUM 6
#define N_ORDERS 14
#define STR_SIZE 7
#define HOME_CNT 32767
#define MIN_DUTY 0
#define HALF_DUTY 2047
#define MAX_DUTY 4095
#define ENCMAX 65535
#define ENCMIN 0
#define SECTOR3_ADDR 0x0800C000
#define MEMBUFF 500

UART_HandleTypeDef huart2;

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
TIM_HandleTypeDef htim15;

extern char orders[N_ORDERS][STR_SIZE];
extern char split[N_STRING][BUFF_SIZE];
extern uint32_t enco[6], position[6], velo[10], encoActual[6];
extern float32_t pwm[6];
extern long int error[6];
extern uint8_t mode;

/*
 * Checks End Of Line \n
 */
uint8_t eol(const char *str) {
	int n = strlen(str);
	if (str[n - 1] != '\n')
		return 1;
	return 0;
}

/*
 * Prints Encoder Values
 */
void showEnco(const uint32_t *arr) {
	char encoders[64], tmp[7];
	int aux;
	encoders[0] = '\0';
	for (int i = 0; i < 5; i++) {
		aux = arr[i] - 32767;
		itoa(aux, tmp, 10);
		strcat(encoders, tmp);
		if (i < 4)
			strcat(encoders, " ");
	}
	/*printf("%s", "Encoder Values Are:");
	 printf("%s\r\n", encoders);*/
	printf("%s\r\n", "Encoder Values Are:");
	for (int i = 0; i < 5; i++)
		printf("%s%d%s%ld\r\n", "Encoder ", i + 1, ": ", arr[i] - 32767);
}

/*
 * Returns The Order Position At "orders" Array.
 * If The Returned Number Is Greater Or Equal To N_ORDERS, The Order Does Not Exist.
 */
uint8_t checkOrder(const char *str) {
	int i, j = 0, k;
	for (i = 0; i < N_ORDERS; i++) {
		k = strncmp(str, orders[i], STR_SIZE);
		if (k != 0)
			j++;
		else
			break;
	}
	return j;
}

/**
 * Checks The Received Command Syntax.
 * If The Returned Value Is Equal To Zero, There Are No Syntax Errors.
 */
uint8_t checkSyntax(char *str) {
	uint8_t n = 0;
	for (int i = 7; i < strlen(str); i++) {
		if ((str[i] < '0' || str[i] > '9')
				&& (str[i] != '\r' && str[i] != '\n' && str[i] != ' '
						&& str[i] != '-' && str[i] != ','))
			n++;
	}
	return n;
}

/*
 * Checks Syntax Of Orders: AT+VELO, AT+LOAD & AT+SAVE
 */
uint8_t checkOneNumSyntax(char *str) {
	uint8_t n = 0, i = 8;

	while (str[i] != '\0') {
		if ((str[i] < '0' || str[i] > '9') && str[i] != '\n')
			n++;
		i++;
	}
	return n;
}

/**
 * Checks AT+MOVE Command Syntax.
 * If The Returned Value Is Equal To Zero, AT+MOVE Order Has No Syntax Errors.
 */
uint8_t checkMoveSyntax(char *str) {
	uint8_t n = 0, j = 0, k = 0, l = 0;
	for (int i = 7; i < strlen(str); i++) {
		if ((str[i] < '0' || str[i] > '9')
				&& (str[i] != '\r' && str[i] != '\n' && str[i] != ' '
						&& str[i] != '-' && str[i] != ','))
			n++;
		if (str[i] == ',')
			j++;
		if (str[i] == '-' && str[i + 1] == '-')
			k++;
		if ((str[i] >= '0' && str[i] <= '9') && str[i + 1] == '-'
				&& (str[i + 2] >= '0' || str[i + 2] <= '9'))
			l++;
	}
	if (j != 4 || k != 0 || l != 0)
		n = n + j + k + l;
	return n;
}

/**
 * It Splits The String And Returns An Integer Array Which Elements Are The Encoder Desired Positions.
 */
int* splitStr(int *dst, char *src) {

	//int count = 0, j = 0;
	uint8_t inRange = 0, count = 0, j = 0;
	int array[5];
	char split[N_STRING][BUFF_SIZE];

	for (int i = 8; i < strlen(src); i++) {
		if (src[i] == ' ' && src[i + 1] == ' ')
			i++;
		if (src[i] == ',' || src[i] == '\0') {
			split[count][j] = '\0';
			count++;
			j = 0;
		} else {
			split[count][j] = src[i];
			j++;
		}
	}
	if (count == 4) {
		for (int i = 0; i < NUM; i++) {
			array[i] = atoi(split[i]);
			if (array[i] > 32767 || array[i] < -32767) {
				printf("%s\r\n", "ERROR!!!\r\nNumber Out Of Range.");
				inRange = 1;
			} else {
				array[i] += 32767;
			}
		}
		if (inRange == 0)
			for (int i = 0; i < 5; i++)
				dst[i] = array[i];
	} else if (count != 4)
		printf("%s\r\n",
				"ERROR!!!\r\nWrong Number Of Arguments Or Syntax Error.");
	return 0;
}

/*
 * Adjusts Maximum & Minimum Speed Of All Motors But The Grip Motor.
 */
uint32_t setSpeed(uint32_t *dst, uint8_t N) {
	N = N >> 1;
	for (int i = 0; i < 5; i++) {
		dst[i] = (N + 50) * MAX_DUTY / 100;
		dst[i + 5] = (50 - N) * MAX_DUTY / 100;
	}
	return 0;
}

/*
 * Copies The Data From Flash Memory Into The Position Array.
 */
uint32_t load(uint32_t *dst, uint8_t N, uint16_t words) {

	uint32_t start = SECTOR3_ADDR + (N * 20);
	if (*(__IO uint32_t*) start == 0xFFFFFFFF) {
		printf("%s\r\n", "Memory Is Empty");
		return 0;
	}
	for (uint16_t i = 0; i < words; i++) {
		dst[i] = *(__IO uint32_t*) start;
		start += 4;
	}
	if (words == 5)
		printf("%s\r\n", "Position Loaded");
	return 0;
}

/*
 * Saves The Current Position Of The Robot.
 */

uint32_t save(uint32_t *src, uint8_t N) {

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError, startAddr = SECTOR3_ADDR;
	uint32_t arr[MEMBUFF];
	HAL_FLASH_Unlock();

	load(arr, 0, MEMBUFF);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_3;
	EraseInitStruct.NbSectors = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
		return HAL_FLASH_GetError();
	}

	for (uint16_t i = 0; i < MEMBUFF; i++) {
		if (i == (N * 5)) {
			for (uint8_t j = 0; j < 5; j++)
				arr[i + j] = src[j];
			i += 5;
		}
	}
	for (uint16_t i = 0; i < MEMBUFF; i++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddr, arr[i])
				== HAL_OK) {
			startAddr += 4;
		} else {
			return HAL_FLASH_GetError();
		}
	}
	HAL_FLASH_Lock();
	printf("%s\r\n", "Position Saved Successfully");
	return 0;
}

/*
 * Clear Flash Memory Sector Where Robot Positions Are Stored
 */

uint32_t clearFlash() {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t home_pos[5] = { HOME_CNT, HOME_CNT, HOME_CNT, HOME_CNT, HOME_CNT },
			SectorError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_3;
	EraseInitStruct.NbSectors = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
		return HAL_FLASH_GetError();
	}
	save(home_pos, 0);
	return 0;
}

/*
 * Prints A List Of N elements That Shows The Current Status Of
 */
void printList(uint8_t N) {

	uint32_t arr[MEMBUFF];
	load(arr, 0, MEMBUFF);
	for (uint8_t i = 0; i < 100; i++) {
		if (arr[20 * i] == 0xFFFFFFFF)
			printf("%d%s", i, " : Empty     ");
		else if (arr[20 * i] != 0xFFFFFFFF)
			printf("%d%s", i, " : Filled    ");
		if (i % 5 == 0)
			printf("%s", "\r\n");
	}
}

/*
 * Converts A String Into An Integer Number
 */
uint16_t getNum(char *str) {
	char arr[3];
	uint8_t i = 8, j = 0;
	uint16_t n;
	while (str[i] != '\n') {
		if (str[i] == ' ')
			i++;
		else {
			arr[j] = str[i];
			i++;
			j++;
		}
	}
	arr[j] = '\0';
	n = atoi(arr);
	return n;
}

/*
 * Resets 3 Arrays (enco, position & encoder timers) To HOME_CNT Position
 */

void resetPositions(uint32_t *v1, uint32_t *v2, TIM_HandleTypeDef *v3) {
	for (uint8_t i = 0; i < 5; i++) {
		v1[i] = HOME_CNT;
		v2[i] = HOME_CNT;
		v3[i].Instance->CNT = HOME_CNT;
	}
}

/*
 * Changes Between Terminal Control & Gamepad Control
 */
uint8_t controlMode(uint8_t N) {
	if(N == 127)
		N = 0;
	if(N % 2 != 0)
		printf("%s\r\n", "Terminal Mode Enabled");
	else
		printf("%s\r\n", "Gamepad Mode Enabled");
	return N + 1;
}

/*
 * Open The Grip
 */

void openGrip(TIM_HandleTypeDef *htimx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) {
		pwm[5] = 3000;
		htimx->Instance->CCR1 = pwm[5];
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		printf("%s\r\n", "Grip Opened");
		for (uint32_t i = 0; i < 7500000; i++)
			asm("NOP");
		htimx->Instance->CCR1 = 2047;
		pwm[5] = htimx->Instance->CCR1;
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	}
}

/*
 * Close The Grip
 */

void closeGrip(TIM_HandleTypeDef *htimx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) {
		pwm[5] = 1095;
		htimx->Instance->CCR1 = pwm[5];
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		printf("%s\r\n", "Grip Closed");
		for (uint32_t i = 0; i < 7500000; i++)
			asm("NOP");
		htimx->Instance->CCR1 = 2047;
		pwm[5] = htimx->Instance->CCR1;
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	}
}

/*
 * Enable Robot
 */
void startRobot(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	printf("%s\r\n", "Robot Enabled!!!");
}

/*
 * Disable Robot
 */
void stopRobot(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	printf("%s\r\n", "Robot Disabled!!!");
}

/*
 * Calculates The Duty Cycles For Each Motor.
 */

void getPwm(arm_pid_instance_f32 *pid) {

	// Encoder Timers Array
	TIM_HandleTypeDef encoTimers[6] =
			{ htim1, htim2, htim3, htim4, htim5, htim8 };
	// Pwm Timers Array
	TIM_HandleTypeDef pwmTimers[6] = { htim10, htim11, htim12, htim12, htim13,
			htim14 };

	for (uint8_t i = 0; i < 5; i++) {
		enco[i] = encoTimers[i].Instance->CNT;
		error[i] = enco[i] - position[i];
		pwm[i] = arm_pid_f32(&pid[i], error[i]);
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
	}

}

/*
 * Pre-Home Moves The Robot To A Position Closer To The Home Position
 */
void preHome() {

	/* Pwm Timers Array */
	TIM_HandleTypeDef pwmTimers[6] = { htim10, htim11, htim12, htim12, htim13,
			htim14 };

	/* Switches Arrays */
	GPIO_TypeDef *gpios[5] = { GPIOA, GPIOB, GPIOB, GPIOB, GPIOC };
	uint16_t gpioPorts[5] = { GPIO_PIN_15, GPIO_PIN_1, GPIO_PIN_2,
	GPIO_PIN_10,
	GPIO_PIN_0 };

	TIM10->CCR1 = 512;
	HAL_Delay(1500);
	TIM11->CCR1 = 3583;
	TIM12->CCR1 = 512;
	HAL_Delay(3000);
	TIM10->CCR1 = 2048;
	TIM12->CCR1 = 3583;
	HAL_Delay(1000);
	TIM12->CCR1 = 2048;
	HAL_Delay(2000);
	pwmTimers[3].Instance->CCR2 = 4095 - 1024;
	pwmTimers[4].Instance->CCR1 = 1024;
	while (HAL_GPIO_ReadPin(gpios[4], gpioPorts[4]))
		asm("NOP");
	pwmTimers[3].Instance->CCR2 = 512;
	while (HAL_GPIO_ReadPin(gpios[3], gpioPorts[3]))
		asm("NOP");
}

