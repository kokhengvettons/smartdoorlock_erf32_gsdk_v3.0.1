/***************************************************************************//**
 * @file
 * @brief Driver for the control DC motor
 * sensor
 ******************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

#include "sl_status.h"
#include "sl_pwm.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONTROL_PH_PORT               gpioPortA
#define MOTOR_CONTROL_PH_PIN                8
#define MOTOR_ENABLE_SLEEP_PORT             gpioPortA
#define MOTOR_ENABLE_SLEEP_PIN              3
#define MOTOR_CURRENT_IPROPI_PORT           gpioPortC
#define MOTOR_CURRENT_IPROPI_PIN            6
#define MOTOR_OUTPUT_nFAULT_PORT            gpioPortC
#define MOTOR_OUTPUT_nFAULT_PIN             7
#define MOTOR_CONTROL_PWM_PORT              gpioPortA
#define MOTOR_CONTROL_PWM_PIN               7

#define MOTOR_PWM_FREQUENCY                 26600
#define MOTOR_PWM_POLARITY                  PWM_ACTIVE_HIGH
#define MOTOR_PWM_PERIPHERAL                TIMER0
#define MOTOR_PWM_PERIPHERAL_NO             0
#define MOTOR_PWM_OUTPUT_CHANNEL            0

#define MOTOR_CONTROL_INTERVAL_MS           1000

/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void motor_gpio_init(void);

/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t motor_control_init(void);

/***************************************************************************//**
 *    Enable the motor driver
 ******************************************************************************/
void motor_driver_enable(bool bEnable);

/***************************************************************************//**
 *    Enable the motor driver
 ******************************************************************************/
void motor_direction_control(bool bClockWise);

/***************************************************************************//**
 *    Enable the motor driver
 ******************************************************************************/
sl_status_t motor_pwm_init(bool bInit);

/***************************************************************************//**
 *    motor execute lock operation
 ******************************************************************************/
sl_status_t motor_exec_lock(bool bEnableLock);

/***************************************************************************//**
 *    read the motor fault value
 ******************************************************************************/
bool motor_fault_indicator_read(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
