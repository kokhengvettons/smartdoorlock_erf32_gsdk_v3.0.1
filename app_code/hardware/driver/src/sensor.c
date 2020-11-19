/***************************************************************************//**
 * @file
 * @brief Driver for door sensor
 ******************************************************************************/
#include "sl_app_assert.h"
#include "sl_bt_api.h"
#include "gatt_db.h"
#include "gpiointerrupt.h"
#include "doorlock.h"
#include "em_gpio.h"
#include "sensor.h"
#include "motor.h"
#include "app.h"

const uint8_t door_open_str[4]                = {"OPEN"};
const uint8_t door_closed_str[6]              = {"CLOSED"};

static door_lock_status_TypeDef door_lock_status;

static bool door_lock_is_running;

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void sensor_gpio_init(void)
{
  GPIO_PinModeSet(SENSOR_DOOR_LOCK_POS_1_PORT, SENSOR_DOOR_LOCK_POS_1_PIN,
                  gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(SENSOR_DOOR_LOCK_POS_2_PORT, SENSOR_DOOR_LOCK_POS_2_PIN,
                  gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(SENSOR_DOOR_LOCK_POS_3_PORT, SENSOR_DOOR_LOCK_POS_3_PIN,
                  gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(SENSOR_DOOR_OPEN_PORT, SENSOR_DOOR_OPEN_PIN,
                  gpioModeInputPullFilter, 1);
}

/***************************************************************************//**
 *   Initialize sensor modules
 ******************************************************************************/
sl_status_t sensor_init(void)
{
  sensor_gpio_init();

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   get the door sensor value
 ******************************************************************************/
door_status_TypeDef sensor_get_door_open(void)
{
  if (GPIO_PinInGet(SENSOR_DOOR_OPEN_PORT, SENSOR_DOOR_OPEN_PIN) == DOOR_CLOSED)
    return DOOR_CLOSED;
  else
    return DOOR_OPEN;
}

/***************************************************************************//**
 *   read the door lock position sensor value
 ******************************************************************************/
uint8_t sensor_read_door_lock_position(uint8_t sensorNum)
{
  if (sensorNum == 1)
    return GPIO_PinInGet(SENSOR_DOOR_LOCK_POS_1_PORT, SENSOR_DOOR_LOCK_POS_1_PIN);
  else if (sensorNum == 2)
    return GPIO_PinInGet(SENSOR_DOOR_LOCK_POS_2_PORT, SENSOR_DOOR_LOCK_POS_2_PIN);
  else if (sensorNum == 3)
    return GPIO_PinInGet(SENSOR_DOOR_LOCK_POS_3_PORT, SENSOR_DOOR_LOCK_POS_3_PIN);
  else
    return 0;  
}

/***************************************************************************//**
 *   get door lock status for 3 door lock position sensors 
 ******************************************************************************/
door_lock_status_TypeDef sensor_get_door_lock_status(void)
{
  if (sensor_read_door_lock_position(3) == 1)
    return DOOR_LOCK_FULL;
  else if (sensor_read_door_lock_position(1) != 1)
    return DOOR_LOCK_OPEN;
  else
    return DOOR_LOCK_HALF;
}

/***************************************************************************//**
 *   enable door open sensor interrupt
 ******************************************************************************/
void sensor_door_open_interrupt_enable(void)
{
  // configure PC01 as falling & raising edge trigger on GPIO interrupt source 1
  // for keypad event
  GPIO_ExtIntConfig(SENSOR_DOOR_OPEN_PORT, SENSOR_DOOR_OPEN_PIN,
                    INT_SOURCE_DOOR_SENSOR, true, true, true);

  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_SENSOR,
                           (GPIOINT_IrqCallbackPtr_t)sensor_door_open_handler);
}

/***************************************************************************//**
 *   enable door lock position sensor interrupt
 ******************************************************************************/
void sensor_door_lock_position_interrupt_enable(void)
{
  GPIO_ExtIntConfig(SENSOR_DOOR_LOCK_POS_1_PORT, SENSOR_DOOR_LOCK_POS_1_PIN,
                    INT_SOURCE_DOOR_LOCK_POS_1, true, false, true);
  GPIO_ExtIntConfig(SENSOR_DOOR_LOCK_POS_2_PORT, SENSOR_DOOR_LOCK_POS_2_PIN,
                    INT_SOURCE_DOOR_LOCK_POS_2, true, false, true);
  GPIO_ExtIntConfig(SENSOR_DOOR_LOCK_POS_3_PORT, SENSOR_DOOR_LOCK_POS_3_PIN,
                    INT_SOURCE_DOOR_LOCK_POS_3, true, false, true);

  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_LOCK_POS_1,
                           (GPIOINT_IrqCallbackPtr_t)sensor_door_lock_position_handler);
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_LOCK_POS_2,
                           (GPIOINT_IrqCallbackPtr_t)sensor_door_lock_position_handler);
  GPIOINT_CallbackRegister(INT_SOURCE_DOOR_LOCK_POS_3,
                           (GPIOINT_IrqCallbackPtr_t)sensor_door_lock_position_handler);
}

/***************************************************************************//**
 *   door open sensor interrupt handler
 ******************************************************************************/
void sensor_door_open_handler(int interrupt_no)
{
  (void) interrupt_no;

  sl_status_t sc;
  uint16_t len = 0;

  if (sensor_get_door_open() == DOOR_OPEN)
  {
    sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_status, sizeof(door_open_str), door_open_str, &len);
    sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to send notification when door open. \n",
              (int)sc);

    doorlock_trigger_alarm_timer(doorlock_get_alarm_time());
  }
  else
  {
    sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_status, sizeof(door_closed_str), door_closed_str, &len);
    sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to send notification when door closed. \n",
              (int)sc);

    doorlock_trigger_auto_lock_timer(doorlock_get_auto_lock_time());
    doorlock_stop_alarm_timer();
  }  
}

/***************************************************************************//**
 *   door open sensor interrupt handler
 ******************************************************************************/
void sensor_door_lock_position_handler(int interrupt_no)
{
  (void) interrupt_no;

  if (door_lock_is_running == true)
  {
    // terminate DC motor when is lock is open
    if (door_lock_status == DOOR_LOCK_FULL)
    {
      if (sensor_get_door_lock_status() == DOOR_LOCK_OPEN)
      {
        door_lock_is_running = false;
        motor_operation_terminate();
      }
    }

    // terminate DC motor when is lock is locked
    if (door_lock_status == DOOR_LOCK_OPEN && 
        sensor_get_door_lock_status() == DOOR_LOCK_FULL)
    {
      door_lock_is_running = false;
      motor_operation_terminate();
    }
  }
}

/***************************************************************************//**
 *   read current door open status
 ******************************************************************************/
sl_status_t sensor_read_request(uint8_t connection, uint16_t characteristic)
{
  sl_status_t sc;
  uint16_t len = 0;

  if (sensor_get_door_open() == DOOR_OPEN)
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_open_str),
        door_open_str, &len);
    sl_app_assert(sc == SL_STATUS_OK,
          "[E: 0x%04x] Failed to send read request when door open. \n",
          (int)sc);
  }
  else
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_closed_str),
        door_closed_str, &len);
    sl_app_assert(sc == SL_STATUS_OK,
          "[E: 0x%04x] Failed to send read request when door closed. \n",
          (int)sc);    
  }

  return sc;
}

/***************************************************************************//**
 *   set door lock operation flag
 ******************************************************************************/
void sensor_set_door_lock_operation(bool bRunning)
{
  door_lock_is_running = bRunning;
  door_lock_status = sensor_get_door_lock_status();
}
