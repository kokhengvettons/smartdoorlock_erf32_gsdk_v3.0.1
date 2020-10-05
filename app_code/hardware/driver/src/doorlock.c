/***************************************************************************//**
 * @file
 * @brief handler door lock operation
 * sensor
 ******************************************************************************/
#include "sl_simple_timer.h"
#include "sl_app_assert.h"
#include "sl_app_log.h"
#include "sl_bt_api.h"
#include "gatt_db.h"
#include "doorlock.h"
#include "motor.h"
#include "sensor.h"
#include "string.h"


const uint8_t door_lock_str[4]            = {"LOCK"};
const uint8_t door_unlock_str[6]          = {"UNLOCK"};
static uint8_t door_lock_status           = DOOR_UNLOCK;
static uint8_t door_alarm_status          = DOOR_ALARM_OFF;
static uint8_t door_auto_lock             = DISABLE_AUTO_LOCK;

static uint32_t door_auto_lock_time_in_s  = MIN_DOOR_AUTO_LOCK_SEC;
static uint32_t door_alarm_time_in_s      = MIN_DOOR_SENSOR_ALARM_SEC;

static bool alarm_timer_is_running        = false;

// simple timer to trigger auto lock the door
static sl_simple_timer_t auto_lock_timer;
static void auto_lock_timer_cb(sl_simple_timer_t *timer, void *data);

// simple timer to trigger door alarm
static sl_simple_timer_t alarm_timer;
static void alarm_timer_cb(sl_simple_timer_t *timer, void *data);

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/**************************************************************************//**
 * Timer callback
 * Called for auto lock the door
 *****************************************************************************/
void auto_lock_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;

  if (doorlock_get_lock_status() == DOOR_UNLOCK)
  {
    if (sensor_get_door_open() == DOOR_CLOSED)
      doorlock_execute(true);
  }    
}

/**************************************************************************//**
 * Timer callback
 * Called for trigger door alarm
 *****************************************************************************/
void alarm_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;

  sl_status_t sc;
  uint16_t len = 0;
  alarm_timer_is_running = false;

  if (sensor_get_door_open() == DOOR_OPEN)
  {
    docklock_set_alarm_status(DOOR_ALARM_ON);

    sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_sensor_alarm_status, sizeof(door_alarm_status),
        &door_alarm_status, &len);
    sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to send notification when alarm on. \n", (int)sc);
    
    // TODO: Turn on Buzzer

  }
  else
  {
    docklock_set_alarm_status(DOOR_ALARM_OFF);
  }
}

/**************************************************************************//**
 * trigger auto door lock timer when unlock status
 *****************************************************************************/
sl_status_t doorlock_trigger_auto_lock_timer(uint32_t trigger_time_sec)
{
  sl_app_log("door lock trigger auto lock timer. \n");

  sl_status_t sc;

  if (doorlock_get_lock_status() == DOOR_UNLOCK)
  {
    sc = sl_simple_timer_start(&auto_lock_timer, trigger_time_sec * 1000,
                               auto_lock_timer_cb, NULL, false);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to create auto lock timer. \n",
                  (int)sc);
  }

  return sc;
}

/**************************************************************************//**
 * trigger alarm timer when door is open
 *****************************************************************************/
sl_status_t doorlock_trigger_alarm_timer(uint32_t trigger_time_sec)
{
  sl_app_log("door lock trigger alarm timer - %d.\n", trigger_time_sec);

  sl_status_t sc;
  sc = sl_simple_timer_start(&alarm_timer, trigger_time_sec * 1000,
                             alarm_timer_cb, NULL, false);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to create alarm timer. \n", (int)sc);

  alarm_timer_is_running = true;

  return sc;
}

/**************************************************************************//**
 * stop the alarm timer when alarm timer is running but not yet timeout
 *****************************************************************************/
sl_status_t doorlock_stop_alarm_timer(void)
{
  sl_status_t sc;
  if (alarm_timer_is_running == true)
  {
    sl_app_log("door lock stop alarm timer. \n");
    
    sc = sl_simple_timer_stop(&alarm_timer);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to stop alarm timer. \n", (int)sc);    
  }
  
  return sc;
}

/***************************************************************************//**
 *   read request handler for door lock
 ******************************************************************************/
sl_status_t doorlock_read_request(uint8_t connection, uint16_t characteristic)
{
  sl_status_t sc;
  uint16_t len = 0;

  sl_app_log("door lock read request. \n");

  if (doorlock_get_lock_status() == DOOR_LOCK)
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_lock_str),
        door_lock_str, &len);
  }
  else
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_unlock_str),
        door_unlock_str, &len);
  }

  return sc;
}

/***************************************************************************//**
 *   write request handler for door lock
 ******************************************************************************/
sl_status_t doorlock_write_request(uint8_t connection, uint16_t characteristic,
                                   uint8_t data[], uint16_t length)
{
  sl_app_log("door lock write request. \n");

  if (length == sizeof(door_lock_str))
  {
    if (memcmp(data, door_lock_str, length) == 0)
    {
      motor_exec_lock(true);
      doorlock_set_lock_status(DOOR_LOCK);
    }
  }
  else if (length == sizeof(door_unlock_str))
  {
    if (memcmp(data, door_unlock_str, length) == 0)
    {
      motor_exec_lock(false);
      doorlock_set_lock_status(DOOR_UNLOCK);

      // kick start the auto door lock timer, and lock the door when time up
      if (doorlock_get_auto_lock_feature() == ENABLE_AUTO_LOCK)
        doorlock_trigger_auto_lock_timer(door_auto_lock_time_in_s);
    }
  }

  sl_status_t sc = sl_bt_gatt_server_send_user_write_response(
      connection, characteristic, SL_STATUS_OK);

  return sc;
}

/***************************************************************************//**
 *   get door lock current lock status
 ******************************************************************************/
door_lock_TypeDef doorlock_get_lock_status(void)
{
  return door_lock_status;
}

/***************************************************************************//**
 *   get current auto lock feature
 ******************************************************************************/
door_auto_lock_TypeDef doorlock_get_auto_lock_feature(void)
{
  return door_auto_lock;
}

/***************************************************************************//**
 *   get current auto lock time in sec
 ******************************************************************************/
uint32_t doorlock_get_auto_lock_time(void)
{
  return door_auto_lock_time_in_s;
}

/***************************************************************************//**
 *   get current alarm time in sec
 ******************************************************************************/
uint32_t doorlock_get_alarm_time(void)
{
  return door_alarm_time_in_s;
}

/***************************************************************************//**
 *   get current door alarm status
 ******************************************************************************/
door_alarm_TypeDef doorlock_get_alarm_status(void)
{
  return door_alarm_status;
}

/***************************************************************************//**
 *   set door lock current lock status
 ******************************************************************************/
void doorlock_set_lock_status(door_lock_TypeDef value)
{
  if (value == DOOR_UNLOCK  || value == DOOR_LOCK)
  {
    door_lock_status = value;
  }
}

/***************************************************************************//**
 *   set door auto lock feature - on or off
 ******************************************************************************/
void doorlock_set_auto_lock_feature(door_auto_lock_TypeDef value)
{
  if (value == DISABLE_AUTO_LOCK || value == ENABLE_AUTO_LOCK)
  {
    door_auto_lock = value;
  }
  else
  {
    door_auto_lock = DISABLE_AUTO_LOCK;
  }  
}

/***************************************************************************//**
 *   set door auto lock timer in Seconds
 ******************************************************************************/
void doorlock_set_auto_lock_time(uint32_t time_in_sec)
{
  if (time_in_sec <= MIN_DOOR_AUTO_LOCK_SEC ||
      time_in_sec >= MAX_DOOR_AUTO_LOCK_SEC)
    door_auto_lock_time_in_s = MIN_DOOR_AUTO_LOCK_SEC;
  else
    door_auto_lock_time_in_s  = time_in_sec;
}

/***************************************************************************//**
 *   set door alarm timer in Seconds
 ******************************************************************************/
void docklock_set_alarm_time(uint32_t time_in_sec)
{
  if (time_in_sec <= MIN_DOOR_SENSOR_ALARM_SEC ||
      time_in_sec >= MAX_DOOR_SENSOR_ALARM_SEC)
    door_alarm_time_in_s = MIN_DOOR_SENSOR_ALARM_SEC;
  else
    door_alarm_time_in_s = time_in_sec;
}

/***************************************************************************//**
 *   set door alarm status
 ******************************************************************************/
void docklock_set_alarm_status(door_alarm_TypeDef status)
{
  if (status == DOOR_ALARM_OFF || status == DOOR_ALARM_ON)
    door_alarm_status = status;
  else
    door_alarm_status = DOOR_ALARM_OFF;
}

/**************************************************************************//**
 * execute door lock operation
 *****************************************************************************/
void doorlock_execute(bool bEnableLock)
{
  sl_status_t sc;
  uint16_t len = 0;

  if (bEnableLock == true)
  {    
    if ((sc = motor_exec_lock(true)) == SL_STATUS_OK)
    {
      doorlock_set_lock_status(DOOR_LOCK);
      sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_lock, sizeof(door_lock_str), door_lock_str, &len);
    }
  }
  else
  {
    if ((sc = motor_exec_lock(false)) == SL_STATUS_OK)
    {
      doorlock_set_lock_status(DOOR_UNLOCK);
      sc = sl_bt_gatt_server_send_characteristic_notification(
          0xFF, gattdb_door_lock, sizeof(door_unlock_str), door_unlock_str, &len);      
    }
    sl_app_assert(
        sc == SL_STATUS_OK,
        "[E: 0x%04x] Failed to send notification when execute door lock. \n",
        (int)sc);
  }
}

/**************************************************************************//**
 * door lock handler when door open button pressed
 *****************************************************************************/
void doorlock_when_button_pressed(void)
{
  sl_app_log("door pressed to lock/unlock the door lock. \n");

  if (doorlock_get_lock_status() == DOOR_UNLOCK)
  {
    doorlock_execute(true);
  }
  else
  {
    doorlock_execute(false);

    // kick start the auto door lock timer, and lock the door when time up
    if (doorlock_get_auto_lock_feature() == ENABLE_AUTO_LOCK)
      doorlock_trigger_auto_lock_timer(door_auto_lock_time_in_s);
  }
}
