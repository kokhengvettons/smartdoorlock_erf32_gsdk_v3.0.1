/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>


#define FACTORY_RESET_INTERVAL_SEC            2       //  2s

#define PS_KEY_BASE                           0x4000

#define INT_SOURCE_KEYPAD_EVENT               1
#define INT_SOURCE_DOOR_SENSOR                2
#define INT_SOURCE_DOOR_LOCK_POS_1            3
#define INT_SOURCE_DOOR_LOCK_POS_2            4
#define INT_SOURCE_DOOR_LOCK_POS_3            5

/*******************************************************************************
 *********************************   ENUM   ************************************
 ******************************************************************************/
enum special_cmd_error_code
{
	special_cmd_success = 0x00,
	special_cmd_err_write_profile = 0xF0,
    special_cmd_err_hardware_keypad = 0xF1,
    special_cmd_err_hardware_battery = 0xF2,
    special_cmd_err_hardware_door_motor = 0xF3,
	special_cmd_unsupported_cmd = 0xFE,
	special_cmd_unknown_err = 0xFF,
};

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

/**************************************************************************//**
 * Retrieve the attribute value from flash and write into attribute
 *****************************************************************************/
void retrieve_attribute_value_from_flash(uint16_t attribute_id);

/**************************************************************************//**
 * Initialized the attribute value when system boot 
 *****************************************************************************/
void attribute_value_init(void);

/**************************************************************************//**
 * Execute factory reset
 *****************************************************************************/
void factory_reset(void);

/**************************************************************************//**
 * overwrite keypad configuration profile
 *****************************************************************************/
void keypad_hardware_test(bool bWriteConfProfile);

/**************************************************************************//**
 * Battery measurement test
 *****************************************************************************/
void battery_measurement_test(void);

/**************************************************************************//**
 * Special command default handler
 *
 *****************************************************************************/
void special_command_default_handler(void);

#endif // APP_H
