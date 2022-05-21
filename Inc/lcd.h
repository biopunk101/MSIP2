#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "stm32f3xx_hal.h"
#include "main.h"

#define LCD_RS_Set() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
#define LCD_RS_Clr() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
#define LCD_RW_Set() HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET);
#define LCD_RW_Clr() HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
#define LCD_EN_Set() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
#define LCD_EN_Clr() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

void LCD_Init(void);
void LCD_WriteDB(uint8_t data);
void LCD_WriteCmd(uint8_t cmd);
void LCD_WriteData(uint8_t data);

void LCD_ClearScreen(void);
void LCD_SetCursor(uint8_t x, uint8_t y);
void LCD_DispStr(uint8_t x, uint8_t y, uint8_t *str);
void LCD_DispMsg(uint8_t *str);
void LCD_DispVersion(uint8_t num);

/*
#define Busy 0x80

#define LCD_RS_1() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
#define LCD_RS_0() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
#define LCD_RW_1() HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET);
#define LCD_RW_0() HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
#define LCD_CS_1() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
#define LCD_CS_0() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

void WriteDataLCD(uint8_t WDLCD);
void WrtieCmdLCD(uint8_t WDLCD, uint8_t chkSt);

uint8_t ReadDataLCD(void);
uint8_t ReadStatusLCD(void);

void LCD_Init();
void DisplayOneChar(uint8_t x, uint8_t y, uint8_t DData);
void DisplayListChar(uint8_t x, uint8_t y, uint8_t *DData);


void WriteDataLCD(unsigned char WDLCD);
void WriteCommandLCD(unsigned char WCLCD, BuysC);
unsigned char ReadDataLCD(void);
unsigned char ReadStatusLCD(void);
void LCDInit(void);
void DisplayOneChar(unsigned char X, unsigned char Y, unsigned char DData);
void DisplayListChar(unsigned char X, unsigned char Y, unsigned char code *DData);
void Delay5Ms(void);
void Delay400Ms(void);
*/

#endif