#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"
#include "i2c.h"
#include "font.h"

/*UI*/
typedef struct{
    float display_speed;
    uint8_t display_mode;

}UI_StateTypedef;

extern UI_StateTypedef UI_Data;

void OLED_Init();
void OLED_NewFrame();
void OLED_ShowFrame();
void OLED_SetPixel(uint8_t x, uint8_t y);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img);
void OLED_Test();
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end);
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data);
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len);
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data);
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h);
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font);
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font);
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

#endif 
