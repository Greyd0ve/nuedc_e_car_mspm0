#include "OLED.h"

void OLED_Init(void) {}
void OLED_Clear(void) {}
void OLED_Update(void) {}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t fontSize)
{
    (void)x;
    (void)y;
    (void)str;
    (void)fontSize;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize)
{
    (void)x;
    (void)y;
    (void)num;
    (void)len;
    (void)fontSize;
}

void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize)
{
    (void)x;
    (void)y;
    (void)num;
    (void)len;
    (void)fontSize;
}
