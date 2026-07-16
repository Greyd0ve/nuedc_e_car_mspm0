#include "app_e_serial.h"
#include "app_config.h"
#include "app_control.h"
#include "app_e_car.h"
#include "app_line.h"
#include "Board_Config.h"
#include "Key.h"
#include "Motor.h"
#include "PWM.h"
#include "Serial.h"
#include <stdint.h>
#include <string.h>

/* 文本帧串口控制台。
 * 帧格式为中括号包裹的逗号字段，例如 [ecar,set,base,18]。
 * 串口解析在前台执行，UART ISR 只负责把字节放入环形缓冲区。
 */
#define E_SERIAL_PACKET_BUF_SIZE 96U                      /* 单帧接收缓冲区长度，单位字节 */
#define E_SERIAL_FIELD_MAX       8U                       /* 单帧最多字段数 */
#define E_SERIAL_FIELD_LEN       16U                      /* 预留字段长度常量，当前不作为数组长度使用 */
#define E_SERIAL_PLOT_PERIOD_MS  ECAR_SERIAL_PLOT_PERIOD_MS /* 串口曲线/遥测输出周期，单位 ms */
#ifndef E_SERIAL_PLOT_ENABLE
#define E_SERIAL_PLOT_ENABLE        0U
#endif
#define E_SERIAL_LINE_PERIOD_MS  200U                     /* 正式模式 [line] 遥测输出周期，单位 ms */
#define E_SERIAL_JOYSTICK_ACK_MS 500U                     /* joystick 忽略提示最小间隔，单位 ms */

extern volatile float g_targetForwardSpeed;
extern volatile float g_targetTurnSpeed;
extern volatile float g_leftSpeed;
extern volatile float g_rightSpeed;
extern volatile float g_forwardSpeed;
extern volatile float g_pwmLimit;
extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineRawMask;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineBlackCount;
extern volatile uint8_t g_lineBadMaskCount;
extern volatile int16_t g_leftEncoderDelta;
extern volatile int16_t g_rightEncoderDelta;
extern volatile int16_t g_leftPwm;
extern volatile int16_t g_rightPwm;
extern volatile uint8_t g_carEnable;

static uint8_t s_serialReady = 0U;
static uint8_t s_receiving = 0U;
static uint8_t s_packetLen = 0U;
static char s_packet[E_SERIAL_PACKET_BUF_SIZE];
static uint16_t s_plotMs = 0U;
static uint16_t s_lineTelemMs = 0U;
static uint32_t s_serialMs = 0U;
static uint32_t s_lastJoystickAckMs = 0U;
static uint8_t s_joystickAcked = 0U;

#if ECAR_BOARD_TEST_MODE
/* 板级测试电机输出默认锁定，必须收到 [test,arm] 才允许直接 PWM。 */
static uint8_t s_boardTestMotorArmed = 0U;
#endif

/* 简单 ASCII 工具函数，避免在 MCU 上依赖 locale 相关库行为。 */
static char ESerial_ToLower(char c)
{
    if (c >= 'A' && c <= 'Z')
    {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static uint8_t ESerial_StrEqualIgnoreCase(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (ESerial_ToLower(*a) != ESerial_ToLower(*b))
        {
            return 0U;
        }
        a++;
        b++;
    }
    return (uint8_t)(*a == '\0' && *b == '\0');
}

static char *ESerial_Trim(char *text)
{
    char *end;

    /* 原地去空格：packet 缓冲区只归解析器使用。 */
    while (*text == ' ' || *text == '\t')
    {
        text++;
    }

    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t'))
    {
        end--;
    }
    *end = '\0';

    return text;
}

static uint8_t ESerial_IsDigit(char ch)
{
    return (uint8_t)((ch >= '0') && (ch <= '9'));
}

static uint8_t ESerial_ParseFloat(const char *text, float *value)
{
    const char *p;
    uint8_t negative = 0U;
    uint8_t hasDigit = 0U;
    uint32_t integerPart = 0U;
    uint32_t fracPart = 0U;
    uint32_t fracScale = 1U;
    float result;

    if ((text == 0) || (value == 0))
    {
        return 0U;
    }

    p = text;

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    if (*p == '-')
    {
        negative = 1U;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    while (ESerial_IsDigit(*p))
    {
        hasDigit = 1U;
        if (integerPart < 1000000U)
        {
            integerPart = integerPart * 10U + (uint32_t)(*p - '0');
        }
        p++;
    }

    if (*p == '.')
    {
        p++;

        while (ESerial_IsDigit(*p))
        {
            hasDigit = 1U;
            if (fracScale < 100000U)
            {
                fracPart = fracPart * 10U + (uint32_t)(*p - '0');
                fracScale *= 10U;
            }
            p++;
        }
    }

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    if ((*p != '\0') || (hasDigit == 0U))
    {
        return 0U;
    }

    result = (float)integerPart;
    if (fracScale > 1U)
    {
        result += (float)fracPart / (float)fracScale;
    }

    if (negative)
    {
        result = -result;
    }

    *value = result;
    return 1U;
}

static int32_t ESerial_RoundToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static void ESerial_SendFixedValue(float value, uint8_t decimals)
{
    int32_t scale = 1;
    int32_t scaled;
    int32_t integerPart;
    int32_t fracPart;
    uint8_t i;

    /* 手动格式化浮点数，避免依赖嵌入式 libc 的 printf 浮点支持。 */
    for (i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * (float)scale + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * (float)scale - 0.5f);
    }

    if (decimals == 0U)
    {
        Serial_Printf("%ld", (long)scaled);
        return;
    }

    integerPart = scaled / scale;
    fracPart = scaled % scale;
    if (fracPart < 0)
    {
        fracPart = -fracPart;
    }

    Serial_Printf("%ld.", (long)integerPart);
    if (decimals == 1U)
    {
        Serial_Printf("%01ld", (long)fracPart);
    }
    else if (decimals == 2U)
    {
        Serial_Printf("%02ld", (long)fracPart);
    }
    else
    {
        Serial_Printf("%03ld", (long)fracPart);
    }
}

static void ESerial_SendSetOkFloat(const char *name, float value, uint8_t decimals)
{
    Serial_Printf("[status,ok,set,%s,", name);
    ESerial_SendFixedValue(value, decimals);
    Serial_SendString("]\r\n");
}

static void ESerial_SendSetOkInt(const char *name, int32_t value)
{
    Serial_Printf("[status,ok,set,%s,%ld]\r\n", name, (long)value);
}

static void ESerial_SendRangeError(const char *name)
{
    Serial_Printf("[status,err,range,%s]\r\n", name);
}

static void ESerial_SendUnknownError(const char *name)
{
    Serial_Printf("[status,err,unknown,%s]\r\n", name);
}

static void ESerial_SendBadPacket(void)
{
    Serial_SendString("[status,err,bad-packet]\r\n");
}

static uint8_t ESerial_ValueInRange(float value, float minVal, float maxVal)
{
    return (uint8_t)(value >= minVal && value <= maxVal);
}

static uint8_t ESerial_SetLapFromFloat(const char *replyName, float value)
{
    int32_t lap;

    lap = ESerial_RoundToInt(value);
    if (lap < 1 || lap > 5)
    {
        ESerial_SendRangeError(replyName);
        return 0U;
    }

    /* 小车运动时拒绝修改目标圈数。 */
    if (!ECar_SetTargetLap((uint8_t)lap))
    {
        Serial_SendString("[status,err,busy]\r\n");
        return 0U;
    }

    ESerial_SendSetOkInt(replyName, lap);
    return 1U;
}

static uint8_t ESerial_SetNamedParam(const char *name, float value)
{
    /* 所有可调参数都做范围限制，防止串口噪声写入危险值。 */
    if (ESerial_StrEqualIgnoreCase(name, "target") ||
        ESerial_StrEqualIgnoreCase(name, "base") ||
        ESerial_StrEqualIgnoreCase(name, "base_speed"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 60.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.base_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.base_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Kp") ||
        ESerial_StrEqualIgnoreCase(name, "line_kp"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 3.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.line_kp = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.line_kp, 2U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Kd") ||
        ESerial_StrEqualIgnoreCase(name, "line_kd"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 8.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.line_kd = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.line_kd, 2U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "Ki") ||
        ESerial_StrEqualIgnoreCase(name, "n") ||
        ESerial_StrEqualIgnoreCase(name, "lap"))
    {
        return ESerial_SetLapFromFloat(name, value);
    }

    if (ESerial_StrEqualIgnoreCase(name, "recover_speed") ||
        ESerial_StrEqualIgnoreCase(name, "recover"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 40.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.recover_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.recover_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_forward") ||
        ESerial_StrEqualIgnoreCase(name, "corner_forward_speed"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 30.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_forward_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.corner_forward_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_turn") ||
        ESerial_StrEqualIgnoreCase(name, "corner_turn_speed"))
    {
        if (!ESerial_ValueInRange(value, -80.0f, 80.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_turn_speed = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.corner_turn_speed, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "turnLimit") ||
        ESerial_StrEqualIgnoreCase(name, "turn_limit"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, 150.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.turn_limit = value;
        ESerial_SendSetOkFloat(name, g_eCarParam.turn_limit, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "lap_pulse_default") ||
        ESerial_StrEqualIgnoreCase(name, "lap_pulse"))
    {
        if (!ESerial_ValueInRange(value, 1000.0f, 30000.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.lap_pulse_default = ESerial_RoundToInt(value);
        ESerial_SendSetOkInt(name, g_eCarParam.lap_pulse_default);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "pwm_limit"))
    {
        if (!ESerial_ValueInRange(value, 0.0f, (float)PWM_MAX_DUTY))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_pwmLimit = value;
        ESerial_SendSetOkFloat(name, g_pwmLimit, 0U);
        return 1U;
    }

    if (ESerial_StrEqualIgnoreCase(name, "corner_pulse") ||
        ESerial_StrEqualIgnoreCase(name, "corner_turn_pulse"))
    {
        if (!ESerial_ValueInRange(value, 60.0f, 260.0f))
        {
            ESerial_SendRangeError(name);
            return 0U;
        }
        g_eCarParam.corner_turn_pulse = (uint16_t)ESerial_RoundToInt(value);
        ESerial_SendSetOkInt(name, (int32_t)g_eCarParam.corner_turn_pulse);
        return 1U;
    }

    ESerial_SendUnknownError(name);
    return 0U;
}

#if ECAR_BOARD_TEST_MODE && ECAR_TEST_MOTOR_ENABLE
static int16_t ESerial_LimitBoardTestPwm(int32_t value)
{
    /* 板级测试 PWM 有独立小限幅，不使用正常 PID 限幅。 */
    if (value > ECAR_BOARD_TEST_PWM_LIMIT)
    {
        return (int16_t)ECAR_BOARD_TEST_PWM_LIMIT;
    }
    if (value < -ECAR_BOARD_TEST_PWM_LIMIT)
    {
        return (int16_t)(-ECAR_BOARD_TEST_PWM_LIMIT);
    }
    return (int16_t)value;
}
#endif

#if ECAR_BOARD_TEST_MODE
static void ESerial_BoardTestStopMotor(void)
{
    /* test stop/lock 必须同时清直接 PWM 状态和闭环目标速度。 */
    g_carEnable = 0U;
    g_targetForwardSpeed = 0.0f;
    g_targetTurnSpeed = 0.0f;
    g_leftPwm = 0;
    g_rightPwm = 0;
    App_Control_ForcePWMZero();
}

static void ESerial_HandleBoardTest(char *fields[], uint8_t fieldCount, uint8_t commandIndex)
{
#if ECAR_TEST_MOTOR_ENABLE
    float leftValue;
    float rightValue;
    int16_t leftPwm;
    int16_t rightPwm;
#endif

    if (fieldCount <= commandIndex)
    {
        ESerial_SendBadPacket();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[commandIndex], "arm") ||
        ESerial_StrEqualIgnoreCase(fields[commandIndex], "unlock"))
    {
#if !ECAR_TEST_MOTOR_ENABLE
        ESerial_BoardTestStopMotor();
        Serial_SendString("[status,err,motor-test-disabled]\r\n");
        return;
#else
        /* arm 不会启动电机，只允许后续显式 pwm 命令输出。 */
        ESerial_BoardTestStopMotor();
        s_boardTestMotorArmed = 1U;
        Serial_SendString("[status,ok,test,armed]\r\n");
        return;
#endif
    }

    if (ESerial_StrEqualIgnoreCase(fields[commandIndex], "lock") ||
        ESerial_StrEqualIgnoreCase(fields[commandIndex], "disarm"))
    {
        /* lock 立即取消测试 PWM 权限并停车。 */
        s_boardTestMotorArmed = 0U;
        ESerial_BoardTestStopMotor();
        Serial_SendString("[status,ok,test,locked]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[commandIndex], "stop"))
    {
        ESerial_BoardTestStopMotor();
        Serial_SendString("[status,ok,test,stop]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[commandIndex], "pwm"))
    {
#if !ECAR_TEST_MOTOR_ENABLE
        ESerial_BoardTestStopMotor();
        Serial_SendString("[status,err,motor-test-disabled]\r\n");
        return;
#else
        if (fieldCount <= (uint8_t)(commandIndex + 2U))
        {
            ESerial_SendBadPacket();
            return;
        }
        if (!s_boardTestMotorArmed)
        {
            /* 未 arm 前禁止直接 PWM 输出。 */
            Serial_SendString("[status,err,test-locked]\r\n");
            return;
        }
        if (!ESerial_ParseFloat(fields[commandIndex + 1U], &leftValue) ||
            !ESerial_ParseFloat(fields[commandIndex + 2U], &rightValue))
        {
            ESerial_SendRangeError("pwm");
            return;
        }

        leftPwm = ESerial_LimitBoardTestPwm(ESerial_RoundToInt(leftValue));
        rightPwm = ESerial_LimitBoardTestPwm(ESerial_RoundToInt(rightValue));

        g_carEnable = 0U;
        g_targetForwardSpeed = 0.0f;
        g_targetTurnSpeed = 0.0f;
        g_leftPwm = leftPwm;
        g_rightPwm = rightPwm;
        Motor_SetPWM(leftPwm, rightPwm);
        Serial_Printf("[status,ok,test,pwm,%d,%d]\r\n", (int)leftPwm, (int)rightPwm);
        return;
#endif
    }
		
		    if (ESerial_StrEqualIgnoreCase(fields[commandIndex], "speed"))
    {
#if !ECAR_TEST_MOTOR_ENABLE
        ESerial_BoardTestStopMotor();
        Serial_SendString("[status,err,motor-test-disabled]\r\n");
        return;
#else
        float forwardValue;
        float turnValue;

        if (fieldCount <= (uint8_t)(commandIndex + 2U))
        {
            ESerial_SendBadPacket();
            return;
        }

        if (!s_boardTestMotorArmed)
        {
            Serial_SendString("[status,err,test-locked]\r\n");
            return;
        }

        if (!ESerial_ParseFloat(fields[commandIndex + 1U], &forwardValue) ||
            !ESerial_ParseFloat(fields[commandIndex + 2U], &turnValue))
        {
            ESerial_SendRangeError("speed");
            return;
        }

        /*
         * 第一阶段速度闭环测试不要太快：
         * forward 限制在 -25 ~ 25 cm/s；
         * turn 限制在 -30 ~ 30 cm/s。
         */
        if (!ESerial_ValueInRange(forwardValue, -25.0f, 25.0f) ||
            !ESerial_ValueInRange(turnValue, -30.0f, 30.0f))
        {
            ESerial_SendRangeError("speed");
            return;
        }

        g_targetForwardSpeed = forwardValue;
        g_targetTurnSpeed = turnValue;
        g_carEnable = 1U;

        App_Control_ResetPID();

        Serial_SendString("[status,ok,test,speed,");
        ESerial_SendFixedValue(g_targetForwardSpeed, 1U);
        Serial_SendString(",");
        ESerial_SendFixedValue(g_targetTurnSpeed, 1U);
        Serial_SendString("]\r\n");
        return;
#endif
    }

    ESerial_SendUnknownError(fields[commandIndex]);
}
#endif

static uint8_t ESerial_SplitFields(char *packet, char *fields[], uint8_t maxFields)
{
    uint8_t count = 0U;
    char *p;

    if (packet == 0 || fields == 0 || maxFields == 0U)
    {
        return 0U;
    }

    /* 将逗号替换为字符串结束符，fields 指针直接指向 s_packet 内部。 */
    fields[count++] = ESerial_Trim(packet);
    p = packet;
    while (*p != '\0')
    {
        if (*p == ',')
        {
            *p = '\0';
            if (count >= maxFields)
            {
                return 0U;
            }
            fields[count++] = ESerial_Trim(p + 1);
        }
        p++;
    }

    return count;
}

static void ESerial_SendParamSnapshot(void)
{
    Serial_Printf("[ecar,val,n,%u,base,", ECar_GetTargetLap());
    ESerial_SendFixedValue(g_eCarParam.base_speed, 0U);
    Serial_SendString(",recover,");
    ESerial_SendFixedValue(g_eCarParam.recover_speed, 0U);
    Serial_SendString(",cornerF,");
    ESerial_SendFixedValue(g_eCarParam.corner_forward_speed, 0U);
    Serial_SendString(",cornerT,");
    ESerial_SendFixedValue(g_eCarParam.corner_turn_speed, 0U);
    Serial_SendString(",Kp,");
    ESerial_SendFixedValue(g_eCarParam.line_kp, 2U);
    Serial_SendString(",Kd,");
    ESerial_SendFixedValue(g_eCarParam.line_kd, 2U);
    Serial_SendString(",turnLimit,");
    ESerial_SendFixedValue(g_eCarParam.turn_limit, 0U);
    Serial_Printf(",lapPulse,%ld,cornerPulse,%u,pwmLimit,", (long)g_eCarParam.lap_pulse_default, (unsigned int)g_eCarParam.corner_turn_pulse);
    ESerial_SendFixedValue(g_pwmLimit, 0U);
    Serial_SendString("]\r\n");
}

static void ESerial_SendStateSnapshot(void)
{
    /* 状态快照较长，必须保持在前台串口任务中发送。 */
    Serial_Printf("[ecar,state,state,%u,targetLap,%u,lapCount,%u,cornerCount,%u,runningTimeMs,%lu,",
                  (unsigned int)ECar_GetState(),
                  (unsigned int)ECar_GetTargetLap(),
                  (unsigned int)ECar_GetLapCount(),
                  (unsigned int)ECar_GetCornerCount(),
                  (unsigned long)ECar_GetRunningTimeMs());

    Serial_Printf("lineError,%d,rawMask,%02X,lineMask,%02X,blackCount,%u,badMask,%u,lineValid,%u,",
                  (int)g_lineError,
                  (unsigned int)g_lineRawMask,
                  (unsigned int)g_lineMask,
                  (unsigned int)g_lineBlackCount,
                  (unsigned int)g_lineBadMaskCount,
                  (unsigned int)g_lineValid);

    Serial_SendString("leftCmps,");
    ESerial_SendFixedValue(g_leftSpeed, 1U);
    Serial_SendString(",rightCmps,");
    ESerial_SendFixedValue(g_rightSpeed, 1U);
    Serial_Printf(",leftPwm,%d,rightPwm,%d,faultCode,%u,progress,",
                  (int)g_leftPwm,
                  (int)g_rightPwm,
                  (unsigned int)ECar_GetFaultCode());
    ESerial_SendFixedValue(ECar_GetLapProgress() * 100.0f, 0U);
    Serial_SendString("]\r\n");
}

static void ESerial_HandleSlider(char *fields[], uint8_t fieldCount)
{
    float value;

    if (fieldCount < 3U)
    {
        ESerial_SendBadPacket();
        return;
    }

    if (!ESerial_ParseFloat(fields[2], &value))
    {
        ESerial_SendRangeError(fields[1]);
        return;
    }

    (void)ESerial_SetNamedParam(fields[1], value);
}

static void ESerial_HandleKey(char *fields[], uint8_t fieldCount)
{
    if (fieldCount >= 3U &&
        ESerial_StrEqualIgnoreCase(fields[1], "emergency") &&
        ESerial_StrEqualIgnoreCase(fields[2], "down"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,emergency]\r\n");
        return;
    }

    ESerial_SendUnknownError((fieldCount >= 2U) ? fields[1] : "key");
}

static void ESerial_HandleJoystick(void)
{
    /* joystick 包只偶尔回提示，不参与小车运动控制。 */
    if (!s_joystickAcked ||
        (uint32_t)(s_serialMs - s_lastJoystickAckMs) >= E_SERIAL_JOYSTICK_ACK_MS)
    {
        Serial_SendString("[status,ignored,joystick]\r\n");
        s_lastJoystickAckMs = s_serialMs;
        s_joystickAcked = 1U;
    }
}

static void ESerial_HandleECar(char *fields[], uint8_t fieldCount)
{
    float value;

    if (fieldCount < 2U)
    {
        ESerial_SendBadPacket();
        return;
    }

#if ECAR_BOARD_TEST_MODE
    if (ESerial_StrEqualIgnoreCase(fields[1], "test"))
    {
        ESerial_HandleBoardTest(fields, fieldCount, 2U);
        return;
    }
#endif

    if (ESerial_StrEqualIgnoreCase(fields[1], "start"))
    {
#if ECAR_ENABLE_REMOTE_START
        /* 远程启动必须通过编译期开关显式启用。 */
        ECar_Start();
        Serial_SendString("[status,ok,start]\r\n");
#else
        Serial_SendString("[status,err,start-disabled]\r\n");
#endif
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "stop"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,stop]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "get"))
    {
        ESerial_SendParamSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "status"))
    {
        ESerial_SendStateSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[1], "set"))
    {
        if (fieldCount < 4U)
        {
            ESerial_SendBadPacket();
            return;
        }
        if (!ESerial_ParseFloat(fields[3], &value))
        {
            ESerial_SendRangeError(fields[2]);
            return;
        }
        (void)ESerial_SetNamedParam(fields[2], value);
        return;
    }

    ESerial_SendUnknownError(fields[1]);
}

static void ESerial_HandlePacket(char *packet)
{
    char *fields[E_SERIAL_FIELD_MAX];
    uint8_t fieldCount;

    fieldCount = ESerial_SplitFields(packet, fields, E_SERIAL_FIELD_MAX);
    if (fieldCount == 0U || fields[0][0] == '\0')
    {
        ESerial_SendBadPacket();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "slider"))
    {
        ESerial_HandleSlider(fields, fieldCount);
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "key"))
    {
        ESerial_HandleKey(fields, fieldCount);
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "joystick"))
    {
        ESerial_HandleJoystick();
        return;
    }

#if ECAR_BOARD_TEST_MODE
    if (ESerial_StrEqualIgnoreCase(fields[0], "test"))
    {
        ESerial_HandleBoardTest(fields, fieldCount, 1U);
        return;
    }
#endif

    if (ESerial_StrEqualIgnoreCase(fields[0], "start"))
    {
#if ECAR_ENABLE_REMOTE_START
        /* 远程启动必须通过编译期开关显式启用。 */
        ECar_Start();
        Serial_SendString("[status,ok,start]\r\n");
#else
        Serial_SendString("[status,err,start-disabled]\r\n");
#endif
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "stop"))
    {
        ECar_Stop();
        Serial_SendString("[status,ok,stop]\r\n");
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "get"))
    {
        ESerial_SendParamSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "status"))
    {
        ESerial_SendStateSnapshot();
        return;
    }

    if (ESerial_StrEqualIgnoreCase(fields[0], "ecar"))
    {
        ESerial_HandleECar(fields, fieldCount);
        return;
    }

    ESerial_SendUnknownError(fields[0]);
}

void ECar_Serial_Init(void)
{
    s_serialReady = 1U;
    s_receiving = 0U;
    s_packetLen = 0U;
    s_plotMs = 0U;
    s_lineTelemMs = 0U;
    s_serialMs = 0U;
    s_lastJoystickAckMs = 0U;
    s_joystickAcked = 0U;
#if ECAR_BOARD_TEST_MODE
    s_boardTestMotorArmed = 0U;
#endif
    Serial_Printf("[status,ok,serial,%u]\r\n", (unsigned int)SERIAL_BAUD_RATE);
#if ECAR_BOARD_TEST_MODE
    Serial_SendString("[status,ok,board-test]\r\n");
#endif
}

void ECar_SerialProcess(void)
{
    uint8_t byte;

    if (!s_serialReady)
    {
        return;
    }

    /* 消费 UART 环形缓冲区所有字节；解析逻辑不放进中断。 */
    while (Serial_ReadByte(&byte))
    {
        char ch = (char)byte;

        if (ch == '[')
        {
            /* 收到新的 '[' 时重新同步，可从噪声或残缺帧中恢复。 */
            s_receiving = 1U;
            s_packetLen = 0U;
            continue;
        }

        if (!s_receiving)
        {
            continue;
        }

        if (ch == ']')
        {
            s_packet[s_packetLen] = '\0';
            s_receiving = 0U;
            ESerial_HandlePacket(s_packet);
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            continue;
        }

        if (s_packetLen < (uint8_t)(E_SERIAL_PACKET_BUF_SIZE - 1U))
        {
            s_packet[s_packetLen++] = ch;
        }
        else
        {
            s_receiving = 0U;
            s_packetLen = 0U;
            ESerial_SendBadPacket();
        }
    }
}

static void ESerial_SendRunPlot(void)
{
    Serial_Printf("[plot,%d,%d,%d,%d]\r\n",
                  (int)g_targetForwardSpeed,
                  (int)g_forwardSpeed,
                  (int)g_lineError,
                  (int)g_leftPwm);
}

#if ECAR_BOARD_TEST_MODE
static void ESerial_SendBoardTestTelemetry(void)
{
    static uint8_t phase = 0U;

    /* 板测遥测分 4 种帧轮流输出，避免单次 100ms 发送过长。 */
    switch (phase)
    {
        case 0U:
            Serial_Printf("[key,%u]\r\n", (unsigned int)Key_GetState());
            break;

        case 1U:
            Serial_Printf("[gray,%02X,%02X,%u,%d,%u]\r\n",
                          (unsigned int)g_lineRawMask,
                          (unsigned int)g_lineMask,
                          (unsigned int)g_lineBlackCount,
                          (int)g_lineError,
                          (unsigned int)g_lineValid);
            break;

        case 2U:
            Serial_Printf("[enc,%d,%d,",
                          (int)g_leftEncoderDelta,
                          (int)g_rightEncoderDelta);
            ESerial_SendFixedValue(g_leftSpeed, 1U);
            Serial_SendString(",");
            ESerial_SendFixedValue(g_rightSpeed, 1U);
            Serial_SendString("]\r\n");
            break;

        default:
            Serial_Printf("[pwm,%d,%d,%u]\r\n",
                          (int)g_leftPwm,
                          (int)g_rightPwm,
                          (unsigned int)s_boardTestMotorArmed);
            break;
    }

    phase++;
    if (phase >= 4U)
    {
        phase = 0U;
    }
}
#endif

#if !ECAR_BOARD_TEST_MODE
static void ESerial_SendLineTelem(void)
{
    Serial_Printf("[line,%u,%02X,%u,%u,%d,",
                  (unsigned int)ECar_GetState(),
                  (unsigned int)g_lineMask,
                  (unsigned int)g_lineBlackCount,
                  (unsigned int)g_lineValid,
                  (int)g_lineError);

    ESerial_SendFixedValue(g_targetForwardSpeed, 1U);
    Serial_SendString(",");
    ESerial_SendFixedValue(g_targetTurnSpeed, 1U);
    Serial_SendString(",");
    ESerial_SendFixedValue(g_leftSpeed, 1U);
    Serial_SendString(",");
    ESerial_SendFixedValue(g_rightSpeed, 1U);

    Serial_Printf(",%d,%d,%u,%u,%u]\r\n",
                  (int)g_leftPwm,
                  (int)g_rightPwm,
                  (unsigned int)ECar_GetCornerCount(),
                  (unsigned int)ECar_GetLapCount(),
                  (unsigned int)ECar_GetFaultCode());
}
#endif

static void ESerial_PeriodicTelemetry(uint16_t periodMs)
{
    if (!s_serialReady)
    {
        return;
    }

    /* periodMs 由调用者传入，因此该函数也能用于 10ms 任务累计。 */
    s_serialMs += periodMs;
    s_plotMs += periodMs;
    s_lineTelemMs += periodMs;

    if (s_plotMs >= E_SERIAL_PLOT_PERIOD_MS)
    {
        s_plotMs = 0U;
#if ECAR_BOARD_TEST_MODE
        ESerial_SendBoardTestTelemetry();
#else
#if E_SERIAL_PLOT_ENABLE
        ESerial_SendRunPlot();
#endif
#endif
    }

#if !ECAR_BOARD_TEST_MODE
    if (s_lineTelemMs >= E_SERIAL_LINE_PERIOD_MS)
    {
        s_lineTelemMs = 0U;
        ESerial_SendLineTelem();
    }
#endif
}

void ECar_SerialPlot100ms(void)
{
    ESerial_PeriodicTelemetry(ECAR_SERIAL_PLOT_PERIOD_MS);
}

void ECar_SerialPlot10ms(void)
{
    ESerial_PeriodicTelemetry(10U);
}
