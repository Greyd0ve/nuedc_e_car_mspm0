#ifndef __APP_E_CAR_H
#define __APP_E_CAR_H

#include <stdint.h>

typedef enum
{
    /* 空闲：上电或复位后的停车状态。 */
    E_CAR_IDLE = 0,
    /* 就绪：已选择目标圈数，等待启动。 */
    E_CAR_READY,
    /* 正常循迹：普通赛道段闭环跟线。 */
    E_CAR_LINE_RUN,
    /* 角点进入：一次性记录角点并准备转弯。 */
    E_CAR_CORNER_ENTER,
    /* 角点转弯：通过宽黑线角点时开环转向。 */
    E_CAR_CORNER_TURN,
    /* 角点恢复：低速重新捕获正常黑线。 */
    E_CAR_LINE_RECOVER,
    /* 完成：达到目标圈数，电机保持停止。 */
    E_CAR_FINISH,
    /* 故障：故障锁存，需要复位/停车后才能再次运行。 */
    E_CAR_FAULT
} ECarState_t;

typedef struct
{
    /* 运动速度，单位 cm/s；turn 速度表示左右轮差速命令。 */
    float base_speed;
    float recover_speed;
    float corner_forward_speed;
    float corner_turn_speed;

    /* 循迹误差使用 app_line 权重，输出限幅为 cm/s 转向命令。 */
    float line_kp;
    float line_kd;
    float turn_limit;

    uint16_t lost_timeout_ms;
    uint16_t recover_stable_ms;
    /* 角点转向时间窗口，单位 ms。 */
    uint16_t corner_min_turn_ms;
    uint16_t corner_max_turn_ms;

    /* 编码器距离阈值使用原始脉冲，便于调参和遥测。 */
    int32_t min_corner_interval_pulse;
    int32_t lap_pulse_default;

    /* 判断宽黑色角点标记所需的最小黑色传感器数量。 */
    uint8_t corner_black_count_th;
} ECarParam_t;

extern ECarParam_t g_eCarParam;

/* 上电初始化状态、参数和安全输出。 */
void ECar_Init(void);

/* 清运行数据并进入 READY，不启动电机。 */
void ECar_Reset(void);

/* 启动自动循迹；若编译为板级测试模式则不会进入运行。 */
void ECar_Start(void);

/* 停车、清运行数据并进入 READY。 */
void ECar_Stop(void);

/* 10ms 状态机和控制主入口。 */
void ECar_Control10ms(void);

/* 轮询消抖后的本地按键，并执行状态安全的命令。 */
void ECar_KeyProcess(void);

/* 刷新 OLED 状态显示。 */
void ECar_ShowStatus(void);

/* 只读状态接口，供 OLED、串口和后续瞄准通信模块读取。 */
float ECar_GetLapProgress(void);
uint8_t ECar_GetLapCount(void);
uint8_t ECar_GetTargetLap(void);
ECarState_t ECar_GetState(void);

/* 1ms 非阻塞提示 tick，由定时器 ISR 调用。 */
void ECar_PromptTick1ms(void);

/* 设置目标圈数；仅在小车非运动状态下允许修改。 */
uint8_t ECar_SetTargetLap(uint8_t lap);
uint8_t ECar_GetCornerCount(void);
uint8_t ECar_GetFaultCode(void);
uint32_t ECar_GetRunningTimeMs(void);

#endif
