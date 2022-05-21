#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "stm32f3xx_hal.h"
#include "main.h"

#define BeepOn() HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET)
#define BeepOff() HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET)

void Buzzer(uint16_t ms);

#endif