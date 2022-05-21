#include "buzzer.h"

void Buzzer(uint16_t ms)
{
    BeepOn();
    HAL_Delay(ms);
    BeepOff();
}