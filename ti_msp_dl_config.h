/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_SYS */
#define TIMER_SYS_INST                                                   (TIMG8)
#define TIMER_SYS_INST_IRQHandler                               TIMG8_IRQHandler
#define TIMER_SYS_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
#define TIMER_SYS_INST_LOAD_VALUE                                         (124U)



/* Defines for UART_MAIN */
#define UART_MAIN_INST                                                     UART1
#define UART_MAIN_INST_FREQUENCY                                        32000000
#define UART_MAIN_INST_IRQHandler                               UART1_IRQHandler
#define UART_MAIN_INST_INT_IRQN                                   UART1_INT_IRQn
#define GPIO_UART_MAIN_RX_PORT                                             GPIOA
#define GPIO_UART_MAIN_TX_PORT                                             GPIOA
#define GPIO_UART_MAIN_RX_PIN                                      DL_GPIO_PIN_9
#define GPIO_UART_MAIN_TX_PIN                                      DL_GPIO_PIN_8
#define GPIO_UART_MAIN_IOMUX_RX                                  (IOMUX_PINCM20)
#define GPIO_UART_MAIN_IOMUX_TX                                  (IOMUX_PINCM19)
#define GPIO_UART_MAIN_IOMUX_RX_FUNC                   IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_MAIN_IOMUX_TX_FUNC                   IOMUX_PINCM19_PF_UART1_TX
#define UART_MAIN_BAUD_RATE                                               (9600)
#define UART_MAIN_IBRD_32_MHZ_9600_BAUD                                    (208)
#define UART_MAIN_FBRD_32_MHZ_9600_BAUD                                     (21)





/* Port definition for Pin Group GPIO_MOTOR */
#define GPIO_MOTOR_PORT                                                  (GPIOB)

/* Defines for AIN1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_AIN1_PIN                                      (DL_GPIO_PIN_6)
#define GPIO_MOTOR_AIN1_IOMUX                                    (IOMUX_PINCM23)
/* Defines for AIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_MOTOR_AIN2_PIN                                      (DL_GPIO_PIN_7)
#define GPIO_MOTOR_AIN2_IOMUX                                    (IOMUX_PINCM24)
/* Defines for BIN1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_MOTOR_BIN1_PIN                                      (DL_GPIO_PIN_8)
#define GPIO_MOTOR_BIN1_IOMUX                                    (IOMUX_PINCM25)
/* Defines for BIN2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_MOTOR_BIN2_PIN                                      (DL_GPIO_PIN_9)
#define GPIO_MOTOR_BIN2_IOMUX                                    (IOMUX_PINCM26)
/* Defines for STBY: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_MOTOR_STBY_PIN                                     (DL_GPIO_PIN_23)
#define GPIO_MOTOR_STBY_IOMUX                                    (IOMUX_PINCM51)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for E1A: GPIOB.10 with pinCMx 27 on package pin 62 */
// pins affected by this interrupt request:["E1A","E2A"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_E1A_IIDX                               (DL_GPIO_IIDX_DIO10)
#define GPIO_ENCODER_E1A_PIN                                    (DL_GPIO_PIN_10)
#define GPIO_ENCODER_E1A_IOMUX                                   (IOMUX_PINCM27)
/* Defines for E1B: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_ENCODER_E1B_PIN                                    (DL_GPIO_PIN_11)
#define GPIO_ENCODER_E1B_IOMUX                                   (IOMUX_PINCM28)
/* Defines for E2A: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_ENCODER_E2A_IIDX                                (DL_GPIO_IIDX_DIO0)
#define GPIO_ENCODER_E2A_PIN                                     (DL_GPIO_PIN_0)
#define GPIO_ENCODER_E2A_IOMUX                                   (IOMUX_PINCM12)
/* Defines for E2B: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_ENCODER_E2B_PIN                                     (DL_GPIO_PIN_1)
#define GPIO_ENCODER_E2B_IOMUX                                   (IOMUX_PINCM13)
/* Port definition for Pin Group GPIO_GRAYSCALE */
#define GPIO_GRAYSCALE_PORT                                              (GPIOA)

/* Defines for OUT: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_GRAYSCALE_OUT_PIN                                  (DL_GPIO_PIN_14)
#define GPIO_GRAYSCALE_OUT_IOMUX                                 (IOMUX_PINCM36)
/* Defines for AD0: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_GRAYSCALE_AD0_PIN                                  (DL_GPIO_PIN_15)
#define GPIO_GRAYSCALE_AD0_IOMUX                                 (IOMUX_PINCM37)
/* Defines for AD1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_GRAYSCALE_AD1_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_GRAYSCALE_AD1_IOMUX                                 (IOMUX_PINCM38)
/* Defines for AD2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_GRAYSCALE_AD2_PIN                                  (DL_GPIO_PIN_17)
#define GPIO_GRAYSCALE_AD2_IOMUX                                 (IOMUX_PINCM39)
/* Port definition for Pin Group GPIO_KEYS */
#define GPIO_KEYS_PORT                                                   (GPIOB)

/* Defines for K1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_KEYS_K1_PIN                                        (DL_GPIO_PIN_15)
#define GPIO_KEYS_K1_IOMUX                                       (IOMUX_PINCM32)
/* Defines for K2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_KEYS_K2_PIN                                        (DL_GPIO_PIN_16)
#define GPIO_KEYS_K2_IOMUX                                       (IOMUX_PINCM33)
/* Defines for K3: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_KEYS_K3_PIN                                         (DL_GPIO_PIN_2)
#define GPIO_KEYS_K3_IOMUX                                       (IOMUX_PINCM15)
/* Defines for K4: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_KEYS_K4_PIN                                         (DL_GPIO_PIN_3)
#define GPIO_KEYS_K4_IOMUX                                       (IOMUX_PINCM16)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_SYS_init(void);
void SYSCFG_DL_UART_MAIN_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
