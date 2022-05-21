#include "lcd.h"
#include <string.h>

void LCD_Init(void)
{
    LCD_WriteCmd(0x38);
    HAL_Delay(5);
    LCD_WriteCmd(0x0c);
    HAL_Delay(5);
    LCD_WriteCmd(0x06);
    HAL_Delay(5);
    LCD_WriteCmd(0x01);
    HAL_Delay(5);
}

void LCD_WriteDBUS(uint8_t *d)
{
    HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, d[7]); // DB7
    HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, d[6]);
    HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, d[5]);
    HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, d[4]);
    HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, d[3]);
    HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, d[2]);
    HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, d[1]);
    HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, d[0]);
}

/*
    Write DATA to LCD DB
*/
void LCD_WriteDB(uint8_t data)
{
    uint8_t d = 0;
    uint8_t db[8] = {0};
#if 0
    db[7] = (data & 0x80) >> 7;
    db[6] = (data & 0x40) >> 6;
    db[5] = (data & 0x20) >> 5;
    db[4] = (data & 0x10) >> 4;
    db[3] = (data & 0x08) >> 3;
    db[2] = (data & 0x04) >> 2;
    db[1] = (data & 0x02) >> 1;
    db[0] = (data & 0x01);

    HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, (GPIO_PinState)db[7]); //DB7
    HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (GPIO_PinState)db[6]);
    HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (GPIO_PinState)db[5]);
    HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, (GPIO_PinState)db[4]);
    HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, (GPIO_PinState)db[3]);
    HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, (GPIO_PinState)db[2]);
    HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, (GPIO_PinState)db[1]);
    HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, (GPIO_PinState)db[0]); //DB0
#else
    HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, (GPIO_PinState)((data & 0x80) >> 7)); // DB7
    HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, (GPIO_PinState)((data & 0x40) >> 6));
    HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, (GPIO_PinState)((data & 0x20) >> 5));
    HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, (GPIO_PinState)((data & 0x10) >> 4));
    HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, (GPIO_PinState)((data & 0x08) >> 3));
    HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, (GPIO_PinState)((data & 0x04) >> 2));
    HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, (GPIO_PinState)((data & 0x02) >> 1));
    HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, (GPIO_PinState)(data & 0x01)); // DB0
#endif
}

void LCD_WriteCmd(uint8_t cmd)
{
    LCD_RS_Clr();
    LCD_RW_Clr();
    LCD_EN_Clr();
    LCD_WriteDB(cmd);
    LCD_EN_Set();
    HAL_Delay(5);
    LCD_EN_Clr();
}

void LCD_WriteData(uint8_t data)
{
    LCD_RS_Set();
    LCD_RW_Clr();
    LCD_EN_Clr();
    LCD_WriteDB(data);
    LCD_EN_Set();
    HAL_Delay(5);
    LCD_EN_Clr();
}

void LCD_ClearScreen(void)
{
    LCD_WriteCmd(0x01);
}

void LCD_SetCursor(uint8_t x, uint8_t y)
{
    uint8_t addr;

    if (y == 0)
        addr = 0x00 + x;
    else
        addr = 0x40 + x;
    LCD_WriteCmd(addr | 0x80);
}

void LCD_DispStr(uint8_t x, uint8_t y, uint8_t *str)
{
    LCD_SetCursor(x, y);

    while (*str != '\0')
    {
        LCD_WriteData(*str++);
    }
}

uint8_t MsgBuf[16] = {0x00};
void LCD_DispMsg(uint8_t *str)
{
    if (strlen(MsgBuf) != 0)
        LCD_DispStr(0, 0, MsgBuf);

    LCD_DispStr(0, 1, str);
    memcpy(MsgBuf, str, strlen(str));
}

void LCD_DispVersion(uint8_t num)
{
    char line0[16] = {0x00};
    char line1[16] = {0x00};
    LCD_ClearScreen();
    sprintf(line0, "MSIP:V%d.%d", num / 100, num % 100);
    LCD_DispStr(0, 0, line0);
    sprintf(line1, "%s", __DATE__);
    LCD_DispStr(0, 1, line1);
}