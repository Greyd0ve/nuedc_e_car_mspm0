#ifndef __APP_AIM_LINK_H
#define __APP_AIM_LINK_H

/*
 * 后续 K230 瞄准模块通信接口预留。
 *
 * 当前工程的可用 UART 已用于串口调参，因此本模块默认不接入调度器。
 * 只有在明确分配 K230 通信 UART 或桥接链路后，才调用这些接口。
 */
/* 在分配 UART/链路后初始化瞄准通信状态。 */
void AimLink_Init(void);

/* 可选 100ms 慢任务钩子，后续可用于心跳或状态上报。 */
void AimLink_Task100ms(void);

/* 以简单文本帧发送当前小车状态。 */
void AimLink_SendCarState(void);

/* 后续接收入口；不得启动小车或直接控制电机。 */
void AimLink_OnAimPacket(const char *packet);

#endif
