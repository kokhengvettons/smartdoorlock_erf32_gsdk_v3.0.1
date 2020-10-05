/***************************************************************************//**
 * @file
 * @brief Driver for door Sensor
 * sensor
 ******************************************************************************/

#ifndef SENSOR_H_
#define SENSOR_H_

#include "sl_status.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_DOOR_LOCK_POS_1_PORT         gpioPortA
#define SENSOR_DOOR_LOCK_POS_1_PIN          4
#define SENSOR_DOOR_LOCK_POS_2_PORT         gpioPortA
#define SENSOR_DOOR_LOCK_POS_2_PIN          5
#define SENSOR_DOOR_LOCK_POS_3_PORT         gpioPortA
#define SENSOR_DOOR_LOCK_POS_3_PIN          6
#define SENSOR_DOOR_OPEN_PORT               gpioPortC
#define SENSOR_DOOR_OPEN_PIN                2

typedef enum 
{     
    DOOR_CLOSED = 0,
    DOOR_OPEN = 1,
} door_status_TypeDef;

typedef enum 
{     
    DOOR_LOCK_OPEN = 0,
    DOOR_LOCK_PARTIALLY = 1,
    DOOR_LOCK_COMPLETED = 2,
} door_lock_status_TypeDef;



/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void sensor_gpio_init(void);

/***************************************************************************//**
 *   Initialize sensor modules
 ******************************************************************************/
sl_status_t sensor_init(void);

/***************************************************************************//**
 *   read the door sensor value
 ******************************************************************************/
door_status_TypeDef sensor_read_door_open(void);

/***************************************************************************//**
 *   read the door lock position sensor value
 ******************************************************************************/
uint8_t sensor_read_door_lock_position(uint8_t sensorNum);

/***************************************************************************//**
 *   get door lock status for 3 door lock position sensors 
 ******************************************************************************/
door_lock_status_TypeDef sensor_get_door_lock_status(void);

/***************************************************************************//**
 *   enable door open sensor interrupt
 ******************************************************************************/
void sensor_interrupt_enable(void);

/***************************************************************************//**
 *   door open sensor interrupt handler
 ******************************************************************************/
void sensor_door_open_handler(int interrupt_no);

/***************************************************************************//**
 *   read current door open status
 ******************************************************************************/
sl_status_t sensor_read_door_status(uint8_t connection, uint16_t characteristic);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H_ */
