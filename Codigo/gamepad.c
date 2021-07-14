/*
 *
 */
#include <main.h>

SPI_HandleTypeDef hspi3;

// MAX7219 Configuration Registers
uint8_t brgt[2] = { 0x0A, 0x05 };	// Brightness -> 11/32
uint8_t deco[2] = { 0x09, 0x00 };	// Decode     -> Mode 0
uint8_t norm[2] = { 0x0C, 0x01 };	// Display    -> Normal Mode
uint8_t scan[2] = { 0x0B, 0x03 };	// Scan Limit -> 4 Digits
uint8_t shut[2] = { 0x0C, 0x00 };	// Display    -> Shutdown Mode
uint8_t test[2] = { 0x0F, 0x00 };	// Test Mode  -> Disabled

// Words & Digits
uint8_t cleanWord[4] = { 0x00, 0x00, 0x00, 0x00 };	// All Led Are Off
uint8_t cntlWord[4] = { 0x0D, 0x15, 0x0F, 0x0E }; 	// cntL : Control Mode
uint8_t homeWord[4] = { 0x37, 0x1D, 0x15, 0x4F };	// hone : Home position
uint8_t loadWord[4] = { 0x0E, 0x1D, 0x77, 0x3D };	// LoAd : Load program/position
uint8_t saveWord[4] = { 0x5B, 0x77, 0x3E, 0x4F };	// SAvE : Save program/position
uint8_t startWord[4] = { 0x5B, 0x0F, 0x05, 0x0F }; 	// Strt : Activate H Bridges
uint8_t stopWord[4] = { 0x5B, 0x0F, 0x1D, 0x67 }; 	// StoP : Disable H Bridges
uint8_t zeroWord[4] = { 0x00, 0x00, 0x00, 0x7E };	// Zero (digit)


uint8_t numbers[10] = { 0x7E, 0x30, 0x6D, 0x79, 0x33,	// 0, 1, 2, 3, 4
		0x5B, 0x5F, 0x70, 0x7F, 0x73,					// 5, 6, 7, 8, 9
		};

/*
 *  Count Number of Digits
 */
uint8_t numDigits(uint32_t num) {
	uint8_t d = 0;
	do {
		d++;
		num /= 10;
	} while (num != 0);
	return d;
}

/*
 * Returns The Display Representation Of A Number
 */
uint8_t num2Segment(uint8_t num) {
	return (numbers[num]);
}


/*
 * Initializes MAX7219
 */
void initMatrix() {
	sendData(norm);
	sendData(deco);
	sendData(scan);
	sendData(test);
	sendData(brgt);
}

/*
 * ITOA Style Function
 */
void num2Array(uint8_t *dst, uint16_t number) {
	uint8_t i = 0, N = numDigits(number);

	for (i = 0; i <= N - 1; i++) {
		dst[i] = number % 10;
		number /= 10;
	}
}

/*
 * Swaps The Elements Of An Unsigned Integer Array
 */
void reverse(uint8_t *str) {
	uint8_t aux, N = sizeof(str);
	for (int i = 0; i < N; i++) {
		aux = str[i];
		str[i] = str[N - 1];
		str[N - 1] = aux;
		N--;
	}
}

/*
 *
 */
void seg2Str(uint8_t *dst, uint8_t *src) {
	uint8_t N = sizeof(src);
	for (int i = 0; i < N; i++)
		dst[i] = num2Segment(src[i]);
}

/*
 * Send 1 Byte Over SPI
 */
void sendData(uint8_t *dato) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, dato, 2, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/*
 * Sends A 4 Digit Number Or a 4 Letter Word Over SPI
 */
void sendWord(uint8_t *str) {
	uint8_t buffer[2], dig[4] = { 2, 1, 0, 3 };
	for (int i = 0; i < 4; i++) {
		buffer[0] = dig[i] + 1;
		buffer[1] = str[i];
		sendData(buffer);
	}
}
