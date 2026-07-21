#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>

/* Temporary encoder-only memory corruption diagnostic mode. */
#ifndef ECAR_ENCODER_MINIMAL_DEBUG
#define ECAR_ENCODER_MINIMAL_DEBUG              0
#endif

/* Safety switches. Keep remote start disabled unless deliberately enabled.
 * For IMU-only board tests, set ECAR_TEST_MOTOR_ENABLE = 0 to prevent
 * unexpected motor output during gyro calibration and yaw testing. */
#ifndef ECAR_ENABLE_REMOTE_START
#define ECAR_ENABLE_REMOTE_START                0
#endif
#ifndef ECAR_BOARD_TEST_MODE
#define ECAR_BOARD_TEST_MODE                    1
#endif
#ifndef ECAR_TEST_MOTOR_ENABLE
#define ECAR_TEST_MOTOR_ENABLE                  1
#endif
#ifndef ECAR_TEST_BEEP_ENABLE
#define ECAR_TEST_BEEP_ENABLE                   0
#endif
#ifndef ECAR_TEST_OLED_ENABLE
#define ECAR_TEST_OLED_ENABLE                   0
#endif
#ifndef ECAR_IMU_ENABLE
#define ECAR_IMU_ENABLE                         0U
#endif
#ifndef ECAR_TEST_IMU_ENABLE
#define ECAR_TEST_IMU_ENABLE                    0U
#endif

/* Master OLED switch. Set to 1 only when the display is physically connected.
 * When 0, all OLED_Init / OLED_Clear / ECar_ShowStatus calls are compiled out. */
#ifndef ECAR_OLED_ENABLE
#define ECAR_OLED_ENABLE                        0
#endif

/* Cooperative task periods driven by the 1ms timer ISR. */
#ifndef ECAR_TASK_COUNT_MAX
#define ECAR_TASK_COUNT_MAX                     5U
#endif
#define ECAR_ENCODER_SPEED_PERIOD_MS            5U
#define ECAR_CONTROL_PERIOD_MS                  10U
#define ECAR_SERIAL_PLOT_PERIOD_MS              100U
#define ECAR_OLED_REFRESH_PERIOD_MS             200U

/*
 * Nominal JGA25-370B + 65mm wheel conversion constants.
 * ECAR_ENCODER_PULSE_PER_REV is measured under Encoder.c default
 * A rising-edge counting mode. See app_config.h encoder comment.
 */
#define ECAR_PI_F                               3.1415926f
#define ECAR_WHEEL_DIAMETER_CM                 6.5f
#define ECAR_WHEEL_CIRCUMFERENCE_CM            (ECAR_WHEEL_DIAMETER_CM * ECAR_PI_F)

/*
 * Encoder counting mode:
 * - Current Encoder.c default counts only encoder A rising edges.
 * - Encoder B is sampled only for direction judgement.
 * - ECAR_ENCODER_PULSE_PER_REV is the measured wheel-end pulse count under
 *   this exact counting mode.
 *
 * If Encoder.c is changed to A both-edge counting or AB quadrature counting,
 * this value must be measured again.
 */
#define ECAR_ENCODER_PULSE_PER_REV             367.0f
#define ECAR_CM_PER_PULSE                      (ECAR_WHEEL_CIRCUMFERENCE_CM / ECAR_ENCODER_PULSE_PER_REV)

/* Square track nominal distances. Tune with telemetry after hardware tests. */
#define ECAR_LAP_DISTANCE_CM                    400.0f

/*
 * Corner detection: all-white (8-channel count == 0) for LOST_CONFIRM_MS
 * triggers corner. From the second corner onward, the current side must
 * exceed MIN_STRAIGHT_CM before all-white is recognized as a corner.
 */
#define ECAR_CORNER_ADVANCE_CM                  3.0f
#define ECAR_CORNER_MIN_STRAIGHT_CM             80.0f
#define ECAR_CORNER_LOST_CONFIRM_MS             100U
#define ECAR_LINE_LOST_FAULT_MS                 2500U

#define ECAR_CORNER_ADVANCE_PULSE \
    ((int32_t)(ECAR_CORNER_ADVANCE_CM / ECAR_CM_PER_PULSE + 0.5f))
#define ECAR_CORNER_MIN_STRAIGHT_PULSE \
    ((int32_t)(ECAR_CORNER_MIN_STRAIGHT_CM / ECAR_CM_PER_PULSE + 0.5f))

#define ECAR_DEFAULT_LAP_PULSE                  ((int32_t)((ECAR_LAP_DISTANCE_CM / ECAR_CM_PER_PULSE) + 0.5f))

/* Conservative first-run speed defaults, in cm/s. */
#define ECAR_DEFAULT_BASE_SPEED_CMPS            55.0f
#define ECAR_DEFAULT_RECOVER_SPEED_CMPS         10.0f
#define ECAR_DEFAULT_CORNER_FORWARD_CMPS        10.0f
#define ECAR_DEFAULT_CORNER_TURN_CMPS           12.0f
#define ECAR_DEFAULT_TURN_LIMIT_CMPS            8.0f

#ifndef ECAR_CORNER_CONFIRM_COUNT
#define ECAR_CORNER_CONFIRM_COUNT               2U
#endif
#ifndef ECAR_BOARD_TEST_PWM_LIMIT
#define ECAR_BOARD_TEST_PWM_LIMIT               400
#endif

/* ---- Unified application mode selection ---- */
#define ECAR_APP_MODE_NONE           0U
#define ECAR_APP_MODE_CAR_TUNING     1U
#define ECAR_APP_MODE_VISION_TUNING  2U
#define ECAR_APP_MODE_INTEGRATED     3U

#ifndef ECAR_APP_MODE
#define ECAR_APP_MODE                ECAR_APP_MODE_NONE
#endif

#if (ECAR_APP_MODE > 3U)
#error "ECAR_APP_MODE must be 0 (NONE), 1 (CAR_TUNING), 2 (VISION_TUNING), or 3 (INTEGRATED)"
#endif

/* Derived mode macros — do NOT define these individually */
#define ECAR_CAR_TUNING_MODE        (ECAR_APP_MODE == ECAR_APP_MODE_CAR_TUNING)
#define ECAR_VISUAL_GIMBAL_XY_TEST_MODE (ECAR_APP_MODE == ECAR_APP_MODE_VISION_TUNING)
#define ECAR_E_TASK_MODE            (ECAR_APP_MODE == ECAR_APP_MODE_INTEGRATED)

/* Legacy stub macros for special test modes */
#ifndef ECAR_GIMBAL_STEP_TEST_MODE
#define ECAR_GIMBAL_STEP_TEST_MODE    0U
#endif
#ifndef ECAR_AIM_LINK_TEST_MODE
#define ECAR_AIM_LINK_TEST_MODE       0U
#endif
#ifndef ECAR_VISUAL_GIMBAL_X_TEST_MODE
#define ECAR_VISUAL_GIMBAL_X_TEST_MODE 0U
#endif
#ifndef ECAR_BOARD_TEST_MODE
#define ECAR_BOARD_TEST_MODE          0U
#endif
#ifndef ECAR_ENCODER_MINIMAL_DEBUG
#define ECAR_ENCODER_MINIMAL_DEBUG    0U
#endif

/* Special test modes: require ECAR_APP_MODE == NONE */
#if (ECAR_APP_MODE != ECAR_APP_MODE_NONE)
#if (ECAR_GIMBAL_STEP_TEST_MODE + ECAR_AIM_LINK_TEST_MODE + \
     ECAR_VISUAL_GIMBAL_X_TEST_MODE + ECAR_BOARD_TEST_MODE + \
     ECAR_ENCODER_MINIMAL_DEBUG) != 0U
#error "Special test modes require ECAR_APP_MODE = ECAR_APP_MODE_NONE"
#endif
#endif

/* Special test modes: only one at a time */
#if (ECAR_GIMBAL_STEP_TEST_MODE + ECAR_AIM_LINK_TEST_MODE + \
     ECAR_VISUAL_GIMBAL_X_TEST_MODE + ECAR_BOARD_TEST_MODE + \
     ECAR_ENCODER_MINIMAL_DEBUG) > 1U
#error "Only one special test mode may be active at a time"
#endif

/* NONE mode must have a reason */
#if (ECAR_APP_MODE == ECAR_APP_MODE_NONE) && \
    (ECAR_GIMBAL_STEP_TEST_MODE + ECAR_AIM_LINK_TEST_MODE + \
     ECAR_VISUAL_GIMBAL_X_TEST_MODE + ECAR_BOARD_TEST_MODE + \
     ECAR_ENCODER_MINIMAL_DEBUG) == 0U
#error "ECAR_APP_MODE_NONE requires at least one special test mode"
#endif

/* Visual-to-stepper X-axis parameters. */
#define AIM_X_DEADBAND_PX                       5
#define AIM_X_SOFT_LIMIT_PULSES                 400
#define AIM_X_STEP_FREQ_HZ                      600U
#define AIM_X_MAX_SINGLE_PULSES                 8U
#define AIM_X_LOCK_ENTER_PX                     5
#define AIM_X_LOCK_EXIT_PX                      9
#define AIM_X_LOCK_CONFIRM_FRAMES               4U

#define AIM_X_ERROR_POSITIVE_DIR               1

/* Visual-to-stepper Y-axis parameters. */
#define AIM_Y_DEADBAND_PX                       5
#define AIM_Y_SOFT_LIMIT_PULSES                 250
#define AIM_Y_STEP_FREQ_HZ                      600U
#define AIM_Y_MAX_SINGLE_PULSES                 8U
#define AIM_Y_LOCK_ENTER_PX                     5
#define AIM_Y_LOCK_EXIT_PX                      9
#define AIM_Y_LOCK_CONFIRM_FRAMES               4U

#define AIM_Y_ERROR_POSITIVE_DIR               1

/* Shared visual data age limit. */
#define AIM_XY_DATA_MAX_AGE_MS                  60U

/* E-task: stable aim lock time before starting car tracking. */
#define ECAR_E_TASK_AIM_STABLE_MS               200U

#if (ECAR_E_TASK_AIM_STABLE_MS == 0U)
#error "ECAR_E_TASK_AIM_STABLE_MS must be non-zero"
#endif

/* Visual gimbal XY debug output gate. */
#ifndef ECAR_VISUAL_GIMBAL_XY_DEBUG_ENABLE
#define ECAR_VISUAL_GIMBAL_XY_DEBUG_ENABLE      0U
#endif

/* Visual gimbal debug output gate (legacy X-only). */
#ifndef ECAR_VISUAL_GIMBAL_DEBUG_ENABLE
#define ECAR_VISUAL_GIMBAL_DEBUG_ENABLE         0U
#endif

#if (AIM_X_ERROR_POSITIVE_DIR != 1) && (AIM_X_ERROR_POSITIVE_DIR != -1)
#error "AIM_X_ERROR_POSITIVE_DIR must be 1 or -1"
#endif
#if (AIM_X_DEADBAND_PX < 0)
#error "AIM_X_DEADBAND_PX must not be negative"
#endif
#if (AIM_X_SOFT_LIMIT_PULSES <= 0)
#error "AIM_X_SOFT_LIMIT_PULSES must be positive"
#endif
#if (AIM_X_MAX_SINGLE_PULSES == 0U)
#error "AIM_X_MAX_SINGLE_PULSES must be non-zero"
#endif
#if (AIM_X_LOCK_EXIT_PX < AIM_X_LOCK_ENTER_PX)
#error "AIM_X_LOCK_EXIT_PX must be >= AIM_X_LOCK_ENTER_PX"
#endif
#if (AIM_X_LOCK_CONFIRM_FRAMES == 0U)
#error "AIM_X_LOCK_CONFIRM_FRAMES must be non-zero"
#endif

#if (AIM_Y_ERROR_POSITIVE_DIR != 1) && (AIM_Y_ERROR_POSITIVE_DIR != -1)
#error "AIM_Y_ERROR_POSITIVE_DIR must be 1 or -1"
#endif
#if (AIM_Y_DEADBAND_PX < 0)
#error "AIM_Y_DEADBAND_PX must not be negative"
#endif
#if (AIM_Y_SOFT_LIMIT_PULSES <= 0)
#error "AIM_Y_SOFT_LIMIT_PULSES must be positive"
#endif
#if (AIM_Y_MAX_SINGLE_PULSES == 0U)
#error "AIM_Y_MAX_SINGLE_PULSES must be non-zero"
#endif
#if (AIM_Y_LOCK_EXIT_PX < AIM_Y_LOCK_ENTER_PX)
#error "AIM_Y_LOCK_EXIT_PX must be >= AIM_Y_LOCK_ENTER_PX"
#endif
#if (AIM_Y_LOCK_CONFIRM_FRAMES == 0U)
#error "AIM_Y_LOCK_CONFIRM_FRAMES must be non-zero"
#endif

#if (AIM_X_LOCK_CONFIRM_FRAMES > 255U)
#error "AIM_X_LOCK_CONFIRM_FRAMES exceeds uint8_t counter range"
#endif
#if (AIM_Y_LOCK_CONFIRM_FRAMES > 255U)
#error "AIM_Y_LOCK_CONFIRM_FRAMES exceeds uint8_t counter range"
#endif

/* Stepper motor physical parameters. */
#define GIMBAL_MOTOR_FULL_STEPS_PER_REV         200U
#define GIMBAL_MICROSTEP                        16U
#define GIMBAL_PULSES_PER_REV \
    (GIMBAL_MOTOR_FULL_STEPS_PER_REV * GIMBAL_MICROSTEP)

#define GIMBAL_TEST_ANGLE_DEG                   45U
#define GIMBAL_TEST_PULSES \
    ((GIMBAL_PULSES_PER_REV * GIMBAL_TEST_ANGLE_DEG) / 360U)

#define GIMBAL_TEST_STEP_FREQ_HZ                500U

#define GIMBAL_STEP_FREQ_MIN_HZ                 100U
#define GIMBAL_STEP_FREQ_MAX_HZ                 5000U

#if (AIM_X_STEP_FREQ_HZ < GIMBAL_STEP_FREQ_MIN_HZ) || \
    (AIM_X_STEP_FREQ_HZ > GIMBAL_STEP_FREQ_MAX_HZ)
#error AIM_X_STEP_FREQ_HZ out of range
#endif

#if (AIM_Y_STEP_FREQ_HZ < GIMBAL_STEP_FREQ_MIN_HZ) || \
    (AIM_Y_STEP_FREQ_HZ > GIMBAL_STEP_FREQ_MAX_HZ)
#error AIM_Y_STEP_FREQ_HZ out of range
#endif

/* Direction level configurable per axis. Valid: 0U or 1U. */
#define GIMBAL_X_POSITIVE_DIR_LEVEL             1U
#define GIMBAL_Y_POSITIVE_DIR_LEVEL             1U

/* EN GPIO gate. Set to 1U when hardware wiring is confirmed. */
#ifndef GIMBAL_USE_ENABLE_GPIO
#define GIMBAL_USE_ENABLE_GPIO                  0U
#endif
#define GIMBAL_EN_ACTIVE_LEVEL                  1U

/* Compile-time checks. */
#if (GIMBAL_MICROSTEP == 0U)
#error GIMBAL_MICROSTEP must be non-zero
#endif
#if (GIMBAL_PULSES_PER_REV % 360U) != 0U
/* NOTE: 3200 % 360 = 320, not exact per-degree mapping. */
#endif
#if GIMBAL_TEST_PULSES != 400U
#error GIMBAL_TEST_PULSES must be 400
#endif
#if (GIMBAL_X_POSITIVE_DIR_LEVEL != 0U) && (GIMBAL_X_POSITIVE_DIR_LEVEL != 1U)
#error GIMBAL_X_POSITIVE_DIR_LEVEL must be 0 or 1
#endif
#if (GIMBAL_Y_POSITIVE_DIR_LEVEL != 0U) && (GIMBAL_Y_POSITIVE_DIR_LEVEL != 1U)
#error GIMBAL_Y_POSITIVE_DIR_LEVEL must be 0 or 1
#endif
#if (GIMBAL_EN_ACTIVE_LEVEL != 0U) && (GIMBAL_EN_ACTIVE_LEVEL != 1U)
#error GIMBAL_EN_ACTIVE_LEVEL must be 0 or 1
#endif

#endif
