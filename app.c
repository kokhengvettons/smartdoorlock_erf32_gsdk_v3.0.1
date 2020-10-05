/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
#include "sl_simple_button_instances.h"
#include "sl_i2cspm_instances.h"
#include "sl_simple_timer.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_bt_types.h"
#include "sl_app_log.h"
#include "sl_status.h"
#include "em_common.h"
#include "gatt_db.h"
#include "app.h"

#include "doorlock.h"
#include "cpt212b.h"
#include "battery.h"
#include "motor.h"
#include "sensor.h"

// Connection handle.
static uint8_t app_connection = 0;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// simple timer to reboot the system
static sl_simple_timer_t simple_timer;

/* Global static variable */
const uint8_t fac_rst_device_name[15]     = {"VettonsDoorLock"};
const uint8_t fac_rst_manufact_name[13]   = {"SmartDoorLock"};
const uint8_t fac_rst_auto_lock_time[2]   = {0x3C, 0x00};
const uint8_t fac_rst_door_alarm_time[2]  = {0x1E, 0x00};
const uint8_t fac_rst_sn_string[36]       = {"00000000-0000-0000-0000-000000000000"};
const uint8_t fac_rst_door_auto_lock      = DISABLE_AUTO_LOCK;

/**************************************************************************//**
 * Static function declaration
 *****************************************************************************/
// simple timer callback.
static void simple_timer_cb(sl_simple_timer_t *timer, void *data);

// Battery level indication change callback
static void battery_level_indication_change_cb(
    uint8_t connection, gatt_client_config_flag_t client_config);

// Event for write attribute value and then store it into flash
static void evt_write_attribute_value(
    struct sl_bt_evt_gatt_server_attribute_value_s* attr_val);

// Event for read request - door lock
static void evt_read_request_door_lock(
    struct sl_bt_evt_gatt_server_user_read_request_s* read_req);

// Event for read request - door status
static void evt_read_request_door_status(
    struct sl_bt_evt_gatt_server_user_read_request_s* read_req);

// Event for write request - door lock
void evt_write_request_door_lock(
  struct sl_bt_evt_gatt_server_user_write_request_s* write_req);

// Event for handler special command
static void evt_special_cmd_handler(
    struct sl_bt_evt_gatt_server_attribute_value_s* attr_val);


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  sl_app_log("Vettons City Smart Door Lock initialized\n");

  cpt212b_init(sl_i2cspm_sensor, false);
  motor_control_init();
  sensor_init();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      sl_app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                 evt->data.evt_system_boot.major,
                 evt->data.evt_system_boot.minor,
                 evt->data.evt_system_boot.patch,
                 evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);

      sl_app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 address_type ? "static random" : "public device",
                 address.addr[5],
                 address.addr[4],
                 address.addr[3],
                 address.addr[2],
                 address.addr[1],
                 address.addr[0]);

      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      sl_app_log("Started advertising\n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      sl_app_log("Connection opened\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      sl_app_log("Connection closed\n");

      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      sl_app_log("Started advertising\n");

      battery_terminate_IADC_Measure(ALL_BAT);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_gatt_server_characteristic_status_id:
      switch(evt->data.evt_gatt_server_characteristic_status.characteristic)
      {
        case gattdb_battery_level_cell:
        case gattdb_battery_level_motor:
          battery_level_indication_change_cb(
              evt->data.evt_gatt_server_characteristic_status.connection,
              evt->data.evt_gatt_server_characteristic_status.client_config_flags);
          break;
      }
      break;
    
    case sl_bt_evt_gatt_server_user_write_request_id:
      switch(evt->data.evt_gatt_server_user_write_request.characteristic)
      {
        case gattdb_door_lock:
          evt_write_request_door_lock(&evt->data.evt_gatt_server_user_write_request);
          break;
      }
      break;
          
    case sl_bt_evt_gatt_server_user_read_request_id:
      switch(evt->data.evt_gatt_server_user_read_request.characteristic)
      {
        case gattdb_door_lock:
          evt_read_request_door_lock(&evt->data.evt_gatt_server_user_read_request);
          break;
        case gattdb_door_status:
          evt_read_request_door_status(&evt->data.evt_gatt_server_user_read_request);
          break;
      }
      break;
    
    case sl_bt_evt_gatt_server_attribute_value_id:
      switch(evt->data.evt_gatt_server_attribute_value.attribute)
      {
        case gattdb_device_name:
        case gattdb_manufacturer_name_string:
        case gattdb_serial_number_string:
        case gattdb_door_auto_lock_time:
        case gattdb_enable_auto_door_lock:
        case gattdb_door_sensor_alarm_time:
          evt_write_attribute_value(&evt->data.evt_gatt_server_attribute_value);
          break;
        case gattdb_special_command:
          evt_special_cmd_handler(&evt->data.evt_gatt_server_attribute_value);
          break;
      }
      break;
    
    case sl_bt_evt_system_external_signal_id:
      break;
      
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
  (void)reason;
  (void)connection;
}

/**************************************************************************//**
 * Battery level measurement
 * Indication changed callback
 *
 * Called when indication of battery measurement is enabled/disabled by
 * the client.
 *****************************************************************************/
void battery_level_indication_change_cb(
    uint8_t connection, gatt_client_config_flag_t client_config)
{
  sl_status_t sc;
  app_connection = connection;

  // Indication or notification enabled.
  if (gatt_indication == client_config)
  {
    sc = battery_measure_init(false);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to init battery measurement.\n", (int)sc);
  }
  else if (gatt_disable == client_config)
  {
    battery_terminate_IADC_Measure(ALL_BAT);
  }
}

/**************************************************************************//**
 * Timer callback
 * Called for system reboot when time up.
 *****************************************************************************/
void simple_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;

  sl_bt_system_reset(0);
}

/**************************************************************************//**
 * Door push button to trigger door unlock/lock
 * Button state changed callback
 * @param[in] handle Button event handle
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) 
    {
      doorlock_when_button_pressed();
    }
  }
}

/**************************************************************************//**
 * Event for write attribute value and store it into flash
 *
 *****************************************************************************/
void evt_write_attribute_value(
    struct sl_bt_evt_gatt_server_attribute_value_s* attr_val)
{
  sl_status_t sc;

  uint16_t attribute_id = attr_val->attribute;

  sc = sl_bt_nvm_save(PS_KEY_BASE + attribute_id, attr_val->value.len,
                 attr_val->value.data);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to write attribute into flash\n", (int)sc);

  if (sc == SL_STATUS_OK)
  {
    sc = sl_bt_gatt_server_write_attribute_value(
        attribute_id, 0, attr_val->value.len, attr_val->value.data);

    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to write attribute value\n", (int)sc);
  }

  // update global variable
  if (attribute_id == gattdb_door_auto_lock_time ||
      attribute_id == gattdb_door_sensor_alarm_time)
  {
      // in case user enter in less than 2 bytes value
      uint16_t time_in_s = 0;
      if (attr_val->value.len == 2)
        time_in_s = (attr_val->value.data[1] << 8) + attr_val->value.data[0];
      else
        time_in_s = attr_val->value.data[0];

      if (attribute_id == gattdb_door_auto_lock_time)
      {
        doorlock_set_auto_lock_time(time_in_s);
      }

      if (attribute_id == gattdb_door_sensor_alarm_time)
      {
        docklock_set_alarm_time(time_in_s);
      }
  }

  if (attribute_id == gattdb_enable_auto_door_lock)
    doorlock_set_auto_lock_feature(attr_val->value.data[0]);
}

/**************************************************************************//**
 * Read request event for door lock
 *
 *****************************************************************************/
void evt_read_request_door_lock(
    struct sl_bt_evt_gatt_server_user_read_request_s* read_req)
{
  sl_status_t sc = doorlock_read_request(read_req->connection,
                                         read_req->characteristic);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read request for door lock \n",
                (int)sc);
}

/**************************************************************************//**
 * Read request event for door status
 *
 *****************************************************************************/
static void evt_read_request_door_status(
    struct sl_bt_evt_gatt_server_user_read_request_s* read_req)
{
  sl_status_t sc = sensor_read_request(read_req->connection,
                                      read_req->characteristic);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read request for door status \n",
                (int)sc);
}

/**************************************************************************//**
 * Write request event for door lock
 *
 *****************************************************************************/
void evt_write_request_door_lock(
  struct sl_bt_evt_gatt_server_user_write_request_s* write_req)
{
  sl_status_t sc;
  sc = doorlock_write_request(write_req->connection, write_req->characteristic,
                              write_req->value.data, write_req->value.len);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user write for door lock\n",
                (int)sc);
}

/**************************************************************************//**
 * Retrieve the attribute value from flash and write into attribute
 *
 *****************************************************************************/
void retrieve_attribute_value_from_flash(uint16_t attribute_id)
{
  sl_status_t sc;

  uint8_t max_attr_len;
  size_t  buf_len;
  uint8_t buf[36];
  memset(buf, 0, sizeof(buf));

  switch(attribute_id)
  {
    case gattdb_device_name:
    case gattdb_manufacturer_name_string:
      max_attr_len = 20;
      break;
    case gattdb_serial_number_string:
      max_attr_len = 36;
      break;
    case gattdb_door_auto_lock_time:
    case gattdb_door_sensor_alarm_time:
      max_attr_len = 2;
      break;
    case gattdb_enable_auto_door_lock:
      max_attr_len = 1;
      break;
    default:
      max_attr_len = 1;
      break;
  }

  sc = sl_bt_nvm_load(PS_KEY_BASE + attribute_id, max_attr_len,
                      &buf_len, &buf[0]);
  sl_app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_BT_PS_KEY_NOT_FOUND,
                "[E: 0x%04x] Failed to read attribute from flash\n", (int)sc);

  // if flash content is store nothing then exit
  if (strlen((const char*)buf) == 0)
    return;

  uint8_t buffer[buf_len];
  memcpy(buffer, buf, sizeof(uint8_t)*buf_len);

  if (sc == SL_STATUS_OK)
  {
    sc = sl_bt_gatt_server_write_attribute_value(attribute_id,
                                                 0, sizeof(buffer), buffer);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to write attribute value %d \n",
                  (int)sc, attribute_id);

    // update global variable
    if (attribute_id == gattdb_door_auto_lock_time)
    {
      // in case user enter in less than 2 bytes value
      if (buf_len == 2)
        doorlock_set_auto_lock_time((buf[1] << 8) + buf[0]);
      else
        doorlock_set_auto_lock_time(buf[0]);
    }

    if (attribute_id == gattdb_door_sensor_alarm_time)
    {
      // in case user enter in less than 2 bytes value
      if (buf_len == 2)
        docklock_set_alarm_time((buf[1] << 8) + buf[0]);
      else
        docklock_set_alarm_time(buf[0]);
    }

    if (attribute_id == gattdb_enable_auto_door_lock)
      doorlock_set_auto_lock_feature(buf[0]);
  }
}

/**************************************************************************//**
 * Initialized the attribute value when system boot
 *
 *****************************************************************************/
void attribute_value_init(void)
{
  retrieve_attribute_value_from_flash(gattdb_device_name);
  retrieve_attribute_value_from_flash(gattdb_manufacturer_name_string);
  retrieve_attribute_value_from_flash(gattdb_serial_number_string);
  retrieve_attribute_value_from_flash(gattdb_door_auto_lock_time);
  retrieve_attribute_value_from_flash(gattdb_door_sensor_alarm_time);
  retrieve_attribute_value_from_flash(gattdb_enable_auto_door_lock);
}

/**************************************************************************//**
 * Event handler for special command
 *
 *****************************************************************************/
static void evt_special_cmd_handler(
    struct sl_bt_evt_gatt_server_attribute_value_s* attr_val)
{
  switch (attr_val->value.data[0])
  {
    case 0x01: // factory reset
      factory_reset();
      break;
    case 0x02: // flash keypad configuration profile
      keypad_hardware_test(true);
      break;
    case 0x03: // hardware self test for keypad
      keypad_hardware_test(false);
      break;
    case 0x04: // hardware self test for battery
      battery_measurement_test();
      break;
    case 0x05: // hardware self test for dc motor
      //hardware_self_test_dc_motor();
      break;
    default:
      special_command_default_handler();
      break;
  }
}

/**************************************************************************//**
 * Execute factory reset
 *
 *****************************************************************************/
void factory_reset(void)
{
  sl_app_log("<special command> - factory reset!!\n");
  // Perform a factory reset by erasing PS storage.
  sl_bt_nvm_erase_all();

  // write default device name and alarm trigger time into PS storage.
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_device_name,
                 sizeof(fac_rst_device_name), fac_rst_device_name);
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_manufacturer_name_string,
                 sizeof(fac_rst_manufact_name), fac_rst_manufact_name);
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_door_auto_lock_time,
                 sizeof(fac_rst_auto_lock_time), fac_rst_auto_lock_time);
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_door_sensor_alarm_time,
                 sizeof(fac_rst_door_alarm_time), fac_rst_door_alarm_time);
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_enable_auto_door_lock,
                 sizeof(fac_rst_door_auto_lock), &fac_rst_door_auto_lock);
  sl_bt_nvm_save(PS_KEY_BASE + gattdb_serial_number_string,
                 sizeof(fac_rst_sn_string), fac_rst_sn_string);

  sl_status_t sc;
  uint16_t len = 0;
  uint8_t err_code = special_cmd_success;

  sc = sl_bt_gatt_server_send_characteristic_notification(
      0xFF, gattdb_special_command, 1, &err_code, &len);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send char notification. \n",
                (int)sc);

  // add some delay to reset the device
  sc = sl_simple_timer_start(&simple_timer,
                             FACTORY_RESET_INTERVAL_SEC * 1000,
                             simple_timer_cb, NULL, false);
  sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to create simple timer. \n", (int)sc);
}

/**************************************************************************//**
 * keypad hardware test
 * bool bWriteConfProfile - true for write configuration profile
 *****************************************************************************/
void keypad_hardware_test(bool bWriteConfProfile)
{
  if (bWriteConfProfile == true)
    sl_app_log("<special command> - write configuration profile.\n");
  else
    sl_app_log("<special command> - keypad hardware test.\n");

  sl_status_t sc = cpt212b_init(sl_i2cspm_sensor, bWriteConfProfile);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to init cpt212b keypad.\n", (int)sc);

  uint8_t err_code = special_cmd_success;
  if (sc != SL_STATUS_OK)
  {
    if (bWriteConfProfile == true)
      err_code = special_cmd_err_write_profile;
    else
      err_code = special_cmd_err_hardware_keypad;
  }

  uint16_t len = 0;
  sc = sl_bt_gatt_server_send_characteristic_notification(
      0xFF, gattdb_special_command, 1, &err_code, &len);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send char notification. \n",
                (int)sc);
}

/**************************************************************************//**
 * Battery measurement test
 *
 *****************************************************************************/
void battery_measurement_test(void)
{
  sl_app_log("<special command> - battery measurement test.\n");

  sl_status_t sc = battery_measure_init(true);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to init battery measurement.\n", (int)sc);

}

/**************************************************************************//**
 * Special command default handler
 *
 *****************************************************************************/
void special_command_default_handler(void)
{
  sl_app_log("<special command> - default handler.\n");

  uint16_t len = 0;
  uint8_t err_code = special_cmd_unsupported_cmd;

  sl_status_t sc = sl_bt_gatt_server_send_characteristic_notification(
      0xFF, gattdb_special_command, 1, &err_code, &len);
  sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to send char notification. \n",
              (int)sc);
}

#ifdef SL_CATALOG_CLI_PRESENT
void hello(sl_cli_command_arg_t *arguments)
{
  (void) arguments;
  bd_addr address;
  uint8_t address_type;
  sl_status_t sc = sl_bt_system_get_identity_address(&address, &address_type);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to get Bluetooth address\n",
                (int)sc);
  sl_app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             address_type ? "static random" : "public device",
             address.addr[5],
             address.addr[4],
             address.addr[3],
             address.addr[2],
             address.addr[1],
             address.addr[0]);
}
#endif // SL_CATALOG_CLI_PRESENT
