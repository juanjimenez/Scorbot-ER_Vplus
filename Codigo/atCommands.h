#ifndef INC_ATCOMMANDS_H_
#define INC_ATCOMMANDS_H_

void showEnco(const uint32_t *arr);
uint8_t checkOrder(const char *str);
uint8_t checkMoveSyntax(char *str);
uint8_t checkOneNumSyntax(char *str);
uint8_t checkSyntax(char *str);
int* splitStr(int *dst, char *src);
uint32_t setSpeed(uint32_t *dst, uint8_t N);
uint32_t load(uint32_t *dst, uint8_t N, uint16_t words);
uint16_t getNum(char *str);
uint8_t eol(const char *str);
uint8_t controlMode(uint8_t N);
uint32_t save(uint32_t *src, uint8_t N);
void resetPositions(uint32_t *v1, uint32_t *v2, TIM_HandleTypeDef *v3);
void openGrip(TIM_HandleTypeDef *htimx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void closeGrip(TIM_HandleTypeDef *htimx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void startRobot(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void stopRobot(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void preHome();
void getPwm(arm_pid_instance_f32 *pid);
uint32_t clearFlash();
void printList(uint8_t N);
#endif /* INC_ATCOMMANDS_H_ */
