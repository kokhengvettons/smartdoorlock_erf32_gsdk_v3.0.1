/***************************************************************************//**
 * @file
 * @brief Driver for door sensor
 ******************************************************************************/
#include "sl_app_assert.h"
#include "sl_bt_api.h"
#include "gatt_db.h"
#include "gpiointerrupt.h"
#include "sensor.h"
#include "em_gpio.h"
#include "app.h"


const uint8_t door_open_str[4]                = {"OPEN"};
const uint8_t door_closed_str[6]              = {"CLOSED"};

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
 *   read the door sensor value
 ******************************************************************************/
door_status_TypeDef sensor_read_door_open(void)
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
    return DOOR_LOCK_COMPLETED;
  else if (sensor_read_door_lock_position(1) != 1)
    return DOOR_LOCK_OPEN;
  else
    return DOOR_LOCK_PARTIALLY;
}

/***************************************************************************//**
 *   enable door open sensor interrupt
 ******************************************************************************/
void sensor_interrupt_enable(void)
{
  // configure PC01 as falling edge trigger on GPIO interrupt source 1
  // for keypad event
  GPIO_ExtIntConfig(SENSOR_DOOR_OPEN_PORT, SENSOR_DOOR_OPEN_PIN,
                    INT_SOURCE_DOOR_SENSOR, true, true, true);

  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_SOURCE_KEYPAD_EVENT,
                           (GPIOINT_IrqCallbackPtr_t)sensor_door_open_handler);
}

/***************************************************************************//**
 *   door open sensor interrupt handler
 ******************************************************************************/
void sensor_door_open_handler(int interrupt_no)
{
  (void) interrupt_no;

  sl_status_t sc;
  uint16_t len = 0;

  if (sensor_read_door_open() == DOOR_OPEN)
  {
    sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_status, sizeof(door_open_str), door_open_str, &len);

    // TODO: trigger door sensor alarm when door is opened --> start new timer to calculate the time
  }
  else
  {
    sc = sl_bt_gatt_server_send_characteristic_notification(
        0xFF, gattdb_door_status, sizeof(door_closed_str), door_closed_str, &len);

    // TODO: Enable auto lock

  }
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send notification when door open. \n",
                (int)sc);
  
}

/***************************************************************************//**
 *   read current door open status
 ******************************************************************************/
sl_status_t sensor_read_door_status(uint8_t connection, uint16_t characteristic)
{
  sl_status_t sc;
  uint16_t len = 0;

  if (sensor_read_door_open() == DOOR_OPEN)
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_open_str),
        door_open_str, &len);
  }
  else
  {
    sc = sl_bt_gatt_server_send_user_read_response(
        connection, characteristic, SL_STATUS_OK, sizeof(door_closed_str),
        door_closed_str, &len);
  }

  return sc;
}
