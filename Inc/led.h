#ifndef LED_H
#define LED_H

#include <stdint.h>
#include "stm32f3xx_hal.h"
#include "main.h"

#define LDR_On() HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET)
#define LDR_Off() HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET)
#define LDG_On() HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_SET)
#define LDG_Off() HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_RESET)
#define LDB_On() HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET)
#define LDB_Off() HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET)

void FlashLED(uint8_t t);
void BlinkLED(uint8_t nLED);

#endif