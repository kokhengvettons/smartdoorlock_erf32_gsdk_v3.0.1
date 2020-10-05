/***************************************************************************//**
 * @file
 * @brief handler door lock operation
 * sensor
 ******************************************************************************/

#ifndef DOORLOCK_H_
#define DOORLOCK_H_

#include "sl_status.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_DOOR_AUTO_LOCK_SEC                300     //  300s
#define MIN_DOOR_AUTO_LOCK_SEC                60      //  60s
#define MAX_DOOR_SENSOR_ALARM_SEC             300     //  300s
#define MIN_DOOR_SENSOR_ALARM_SEC             30      //  30s

typedef enum 
{
    DOOR_UNLOCK = 0, 
    DOOR_LOCK = 1,
} door_lock_TypeDef;

typedef enum 
{
    DISABLE_AUTO_LOCK = 0x00, 
    ENABLE_AUTO_LOCK = 0x01
} door_auto_lock_TypeDef;

typedef enum 
{
    DOOR_ALARM_OFF = 0x00, 
    DOOR_ALARM_ON = 0x01
} door_alarm_TypeDef;

/**************************************************************************//**
 * trigger auto door lock timer when unlock status
 *****************************************************************************/
sl_status_t doorlock_trigger_auto_lock_timer(uint32_t trigger_time_sec);

/**************************************************************************//**
 * trigger auto door lock timer when unlock status
 *****************************************************************************/
sl_status_t doorlock_trigger_alarm_timer(uint32_t trigger_time_sec);

/**************************************************************************//**
 * stop the alarm timer when alarm timer is running but not yet timeout
 *****************************************************************************/
sl_status_t doorlock_stop_alarm_timer(void);

/***************************************************************************//**
 *   read request handler for door lock
 ******************************************************************************/
sl_status_t doorlock_read_request(uint8_t connection, uint16_t characteristic);

/***************************************************************************//**
 *   write request handler for door lock
 ******************************************************************************/
sl_status_t doorlock_write_request(uint8_t connection, uint16_t characteristic,
                                   uint8_t data[], uint16_t length);

/***************************************************************************//**
 *   get door lock current lock status
 ******************************************************************************/
door_lock_TypeDef doorlock_get_lock_status(void);

/***************************************************************************//**
 *   get current auto lock feature
 ******************************************************************************/
door_auto_lock_TypeDef doorlock_get_auto_lock_feature(void);

/***************************************************************************//**
 *   get current auto lock time in sec
 ******************************************************************************/
uint32_t doorlock_get_auto_lock_time(void);

/***************************************************************************//**
 *   get current alarm time in sec
 ******************************************************************************/
uint32_t doorlock_get_alarm_time(void);

/***************************************************************************//**
 *   get current door alarm status
 ******************************************************************************/
door_alarm_TypeDef doorlock_get_alarm_status(void);

/***************************************************************************//**
 *   set door lock current lock status
 ******************************************************************************/
void doorlock_set_lock_status(door_lock_TypeDef value);

/***************************************************************************//**
 *   set door auto lock feature - on or off
 ******************************************************************************/
void doorlock_set_auto_lock_feature(door_auto_lock_TypeDef value);

/***************************************************************************//**
 *   set door auto lock timer in Seconds
 ******************************************************************************/
void doorlock_set_auto_lock_time(uint32_t time_in_sec);

/***************************************************************************//**
 *   set door alarm timer in Seconds
 ******************************************************************************/
void docklock_set_alarm_time(uint32_t time_in_sec);

/***************************************************************************//**
 *   set door alarm status
 ******************************************************************************/
void docklock_set_alarm_status(door_alarm_TypeDef status);

/**************************************************************************//**
 * execute door lock operation
 *****************************************************************************/
void doorlock_execute(bool bEnableLock);

/**************************************************************************//**
 * door lock handler when door open button pressed
 *****************************************************************************/
void doorlock_when_button_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* DOORLOCK_H_ */
