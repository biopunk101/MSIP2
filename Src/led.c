#include "led.h"

/*
LD3: Red
LD7: Green
LD4: Blue
*/

void ALL_LED_ON()
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_SET);
}

void ALL_LED_OFF()
{
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_RESET);
}

void FlashLED(uint8_t t)
{
    for (uint8_t i = 0; i < t; i++)
    {
        ALL_LED_ON();
        HAL_Delay(100);
        ALL_LED_OFF();
        HAL_Delay(100);
    }
}

void BlinkLED(uint8_t nLED)
{
    switch (nLED)
    {
    case 1: // Red
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        break;
    case 2: // Green
        HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
        break;
    case 3: // Blue
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        break;
    case 4: // Orange
        HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        break;
    }
}
