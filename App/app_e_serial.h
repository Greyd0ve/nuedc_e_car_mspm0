#ifndef __APP_E_SERIAL_H
#define __APP_E_SERIAL_H

#include <stdint.h>

/* 初始化串口调参状态，并输出启动提示帧。 */
void ECar_Serial_Init(void);

/* 前台解析中括号文本帧。 */
void ECar_SerialProcess(void);

/* 主调度器使用的 100ms 周期遥测入口。 */
void ECar_SerialPlot100ms(void);

/* 可选 10ms 遥测累计入口，供从控制周期驱动串口输出的场景使用。 */
void ECar_SerialPlot10ms(void);

#endif
