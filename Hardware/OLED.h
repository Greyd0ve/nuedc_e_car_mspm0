#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_8X16 16
#define OLED_6X8  8

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t fontSize);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize);
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize);

#endif
