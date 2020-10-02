/***************************************************************************//**
 * @file
 * @brief Driver for door sensor
 ******************************************************************************/

#include "sensor.h"
#include "em_gpio.h"

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
