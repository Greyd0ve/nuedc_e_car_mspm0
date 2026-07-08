#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

void IMU_Init(void);
uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI);
uint8_t IMU_IsReady(void);

#endif
