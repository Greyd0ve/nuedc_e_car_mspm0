#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "ti_msp_dl_config.h"

#define MOTOR_USE_STBY              1U

#define MOTOR_PWM_TIMER_INST        TIMG0
#define MOTOR_PWM_LEFT_CC_INDEX     DL_TIMER_CC_0_INDEX
#define MOTOR_PWM_RIGHT_CC_INDEX    DL_TIMER_CC_1_INDEX
#define MOTOR_PWM_PERIOD_COUNTS     1600U

#define MOTOR_AIN1_PORT             GPIOB
#define MOTOR_AIN1_PIN              DL_GPIO_PIN_6
#define MOTOR_AIN2_PORT             GPIOB
#define MOTOR_AIN2_PIN              DL_GPIO_PIN_7
#define MOTOR_BIN1_PORT             GPIOB
#define MOTOR_BIN1_PIN              DL_GPIO_PIN_8
#define MOTOR_BIN2_PORT             GPIOB
#define MOTOR_BIN2_PIN              DL_GPIO_PIN_9
#define MOTOR_STBY_PORT             GPIOB
#define MOTOR_STBY_PIN              DL_GPIO_PIN_23

#define LEFT_MOTOR_DIR_SIGN         (-1)
#define RIGHT_MOTOR_DIR_SIGN        (+1)

#define ENCODER_LEFT_A_PORT         GPIOB
#define ENCODER_LEFT_A_PIN          DL_GPIO_PIN_10
#define ENCODER_LEFT_B_PORT         GPIOB
#define ENCODER_LEFT_B_PIN          DL_GPIO_PIN_11
#define ENCODER_RIGHT_A_PORT        GPIOB
#define ENCODER_RIGHT_A_PIN         DL_GPIO_PIN_0
#define ENCODER_RIGHT_B_PORT        GPIOB
#define ENCODER_RIGHT_B_PIN         DL_GPIO_PIN_1
#define ENCODER_GPIO_IRQN           GPIOB_INT_IRQn

#define LEFT_ENCODER_SIGN           (-1)
#define RIGHT_ENCODER_SIGN          (+1)

#define GRAYSCALE_OUT_PORT          GPIOA
#define GRAYSCALE_OUT_PIN           DL_GPIO_PIN_14
#define GRAYSCALE_AD0_PORT          GPIOA
#define GRAYSCALE_AD0_PIN           DL_GPIO_PIN_15
#define GRAYSCALE_AD1_PORT          GPIOA
#define GRAYSCALE_AD1_PIN           DL_GPIO_PIN_16
#define GRAYSCALE_AD2_PORT          GPIOA
#define GRAYSCALE_AD2_PIN           DL_GPIO_PIN_17

#define KEY_K1_PORT                 GPIOB
#define KEY_K1_PIN                  DL_GPIO_PIN_15
#define KEY_K2_PORT                 GPIOB
#define KEY_K2_PIN                  DL_GPIO_PIN_16
#define KEY_K3_PORT                 GPIOB
#define KEY_K3_PIN                  DL_GPIO_PIN_2
#define KEY_K4_PORT                 GPIOB
#define KEY_K4_PIN                  DL_GPIO_PIN_3

#define SERIAL_UART_INST            UART1
#define SERIAL_UART_IRQN            UART1_INT_IRQn

#define SYSTEM_TIMER_INST           TIMG8
#define SYSTEM_TIMER_IRQN           TIMG8_INT_IRQn

#endif
