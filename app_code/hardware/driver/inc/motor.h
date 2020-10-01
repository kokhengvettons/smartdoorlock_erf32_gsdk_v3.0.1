/***************************************************************************//**
 * @file
 * @brief Driver for the Battery level measurement
 * sensor
 ******************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

#include "sl_status.h"

#ifdef __cplusplus
extern "C" {
#endif



/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void motor_gpio_init(void);

/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t motor_control_init(void);


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
