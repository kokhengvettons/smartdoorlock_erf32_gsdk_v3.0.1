/***************************************************************************//**
 * @file
 * @brief Driver for the CPT212B Capacitive Sense keypad
 * sensor
 ******************************************************************************/
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_app_log.h"
#include "sl_i2cspm.h"
#include "gpiointerrupt.h"
#include "cpt212b.h"
#include "gatt_db.h"
#include "string.h"
#include "app.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

static uint8_t keypadValue[DOOR_KEY_ACCESS_LEN * 2];
static uint8_t keypadNum = 0;

static void keypad_event_hander(int interrupt_no);

/***************************************************************************//**
 *    Initialize CPT212B Capacitive touch keypad
 ******************************************************************************/
sl_status_t cpt212b_init(sl_i2cspm_t *i2cspm, bool bEnableFlashNewPRofile)
{
  sl_status_t sc;

  sc = SL_STATUS_OK;

  cpt212b_gpio_init();

  if (bEnableFlashNewPRofile == true)
  {
    sc = cpt212b_flash_new_profile(i2cspm);
  }

  if (sc == SL_STATUS_OK)
  {
    // validate configuration profile
    sc = cpt212b_validate_configuration_profile(i2cspm);
    if (sc == SL_STATUS_OK)
    {
      cpt212b_I2C_enable(false);
      sl_sleeptimer_delay_millisecond(10);
      cpt212b_I2C_enable(true);

      // enter sensing mode from configuration loading mode
      sc = cpt212b_enter_sensing_mode(i2cspm);
      sl_sleeptimer_delay_millisecond(10);

      if (sc == SL_STATUS_OK)
        cpt212b_interrupt_enable();
    }
  }

  sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to init cpt212b keypad.\n", (int)sc);

  return sc;
}

/***************************************************************************//**
 *   Initialize GPIO to enable the CPT212B Capacitive touch keypad
 ******************************************************************************/
void cpt212b_gpio_init(void)
{
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN,
                  gpioModeWiredAnd, 1);
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN,
                  gpioModePushPull, 1);

  // enable (active low) to reset
  GPIO_PinOutClear(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN);
  sl_sleeptimer_delay_millisecond(1);

  // disable reset
  GPIO_PinOutSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_RESET_PIN);
  sl_sleeptimer_delay_millisecond(15);

  // enable keypad I2C module
  cpt212b_I2C_enable(true);
}

/***************************************************************************//**
 *   Enable CPT212B Capacitive touch keypad Interrupt
 ******************************************************************************/
void cpt212b_interrupt_enable(void)
{
  // Configure PC01 as input enabled for keypad 
  GPIO_PinModeSet(CPT212B_I2CSENSOR_CONTROL_PORT, CPT212B_I2CSENSOR_ENABLE_PIN,
                  gpioModeInputPull, 1);

  // configure PC01 as falling edge trigger on GPIO interrupt source 1
  // for keypad event
  GPIO_ExtIntConfig(CPT212B_I2CSENSOR_CONTROL_PORT,
                    CPT212B_I2CSENSOR_ENABLE_PIN, 1, false, true, true);

  GPIOINT_Init();
  GPIOINT_CallbackRegister(INT_SOURCE_KEYPAD_EVENT,
                           (GPIOINT_IrqCallbackPtr_t)keypad_event_hander);

  cpt212b_reset_door_access();
}

/***************************************************************************//**
 *   Keypad interrupt handler 
 ******************************************************************************/
void keypad_event_hander(int interrupt_no)
{
  (void) interrupt_no;

  uint8_t eventData[3];
  uint16_t len;

  sl_status_t sc = cpt212b_read_keypad_event(sl_i2cspm_sensor, eventData, 3);
  if (sc != SL_STATUS_OK)
    return;

  int eventTypeValue = eventData[0] & 0x0F;
  if (eventTypeValue == CPT212B_SENSE_EVENT_TOUCH)
  {
    // when pressed # button, send the notification with door passs key to client
    if (eventData[1] == CPT212B_TOUCH_EVENT_CS10)
    {     
      uint8_t key[DOOR_KEY_ACCESS_LEN]; 
      if (keypadNum >= DOOR_KEY_ACCESS_LEN)
      {
        memcpy(key, &keypadValue[keypadNum - DOOR_KEY_ACCESS_LEN],
               sizeof(uint8_t) * DOOR_KEY_ACCESS_LEN);

        sc = sl_bt_gatt_server_send_characteristic_notification(
            0xFF, gattdb_door_password, sizeof(key), key, &len);
        sl_app_assert(sc == SL_STATUS_OK,
                      "[E: 0x%04x] Failed to send char notification. \n",
                      (int)sc);

        //sl_app_log("access code: %s\n", key);
      }
      cpt212b_reset_door_access();
    }
    else
    {
      keypadValue[keypadNum++] = eventData[1];
      if (keypadNum == DOOR_KEY_ACCESS_LEN * 2)
        keypadNum = 0;
    }
  }
  else if (eventTypeValue == CPT212B_SENSE_EVENT_PROXIMITY)
  {
    // TODO: activate the keypad
  }

  // clear the interrupt flags
  GPIO_IntClear(1 << INT_SOURCE_KEYPAD_EVENT);
}


/***************************************************************************//**
 *   Enable I2C module (active low) for CPT212B Capacitive touch keypad
 ******************************************************************************/
void cpt212b_I2C_enable(bool bEnable)
{
  if (bEnable)
    GPIO_PinOutClear(CPT212B_I2CSENSOR_CONTROL_PORT,
                     CPT212B_I2CSENSOR_ENABLE_PIN);
  else
    GPIO_PinOutSet(CPT212B_I2CSENSOR_CONTROL_PORT,
                   CPT212B_I2CSENSOR_ENABLE_PIN);
}

/***************************************************************************//**
 *   Flash a new configuration profile into CPT212B Capacitive touch keypad
 ******************************************************************************/
sl_status_t cpt212b_flash_new_profile(sl_i2cspm_t *i2cspm)
{
  sl_status_t status = SL_STATUS_OK;

  // step 1: Host sends the configuration loading unlock sequence.
  if ((status = cpt212b_loading_unlock_seqence(i2cspm)) == SL_STATUS_OK)
  {
      sl_sleeptimer_delay_millisecond(1);

      // step 2: Host sends config erase command, which erases the configuration
      //         profile.
      if ((status = cpt212b_erase_configuration_profile(i2cspm)) ==
          SL_STATUS_OK)
      {
        sl_sleeptimer_delay_millisecond(200);

        // step 3: Host sends bytes 0-7 of configuration profile in a write
        //         config command, repeats the step until end of profile.
        // Note: The host should pad the last [write bytes] command up to a
        //       payload of 8 bytes, with 0xFF used as padding.
        if ((status = cpt212b_flash_configuration_profile(i2cspm)) ==
            SL_STATUS_OK)
        {
          sl_sleeptimer_delay_millisecond(10);

          // step 4: host sends write CRC Command.
          uint8_t crc2 = CPT212B_A01_GM_DEFAULT_CONFIG_CHECKSUM >> 8;
          uint8_t crc1 = CPT212B_A01_GM_DEFAULT_CONFIG_CHECKSUM & 0xFF;
          status = cpt212b_write_crc_to_flash(i2cspm, crc2, crc1);          
        }
      }
  }

  sl_sleeptimer_delay_millisecond(10);
  
  return status;
}
 
/***************************************************************************//**
 *   Enter configuration loading unlock sequence
 ******************************************************************************/
sl_status_t cpt212b_loading_unlock_seqence(sl_i2cspm_t *i2cspm)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[3];

  i2c_write_data[0] = CPT212B_CONF_UNLOCK;
  i2c_write_data[1] = 0xA5;
  i2c_write_data[2] = 0xF1;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 3;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)
    return SL_STATUS_TRANSMIT;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Erase the configuration profile
 ******************************************************************************/
sl_status_t cpt212b_erase_configuration_profile(sl_i2cspm_t *i2cspm)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1];

  i2c_write_data[0]  = CPT212B_CONF_ERASE;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)  
    return SL_STATUS_TRANSMIT;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Write configuration profile into non-volatile memory
 ******************************************************************************/
sl_status_t cpt212b_flash_configuration_profile(sl_i2cspm_t *i2cspm)
{  
  config_profile_t pf = CPT212B_A01_GM_DEFAULT_CONFIG;

  sl_status_t status;

  // check configuration profile file size
  if ( sizeof(pf) >= CPT212B_MAX_WRITE_CONF_LEN || sizeof(pf) <= 0)
  {
    return SL_STATUS_FLASH_PROGRAM_FAILED;
  }

  // copy configuration profile into buf
  uint8_t buf[CPT212B_MAX_WRITE_CONF_LEN];
  memcpy(&buf[0], pf.reserved_0, sizeof(pf));

  //prepare to write 8-bytes payloads
  uint8_t bytes[CPT212B_FLASH_PAYLOADS];
  for (uint32_t idx = 0; idx < sizeof(pf); idx += CPT212B_FLASH_PAYLOADS)
  {    
    int byte_len = CPT212B_FLASH_PAYLOADS;
    memset(bytes, 0, sizeof(bytes));

    // if write bytes less than 8-bytes payloads, then use oxff as padding for
    // remaining bytes
    if (idx + CPT212B_FLASH_PAYLOADS > sizeof(pf))
    {
      byte_len = sizeof(pf) - idx;
      memset(bytes, 0xff, sizeof(bytes));
    }

    memcpy(bytes, &buf[idx], sizeof(uint8_t) * byte_len);
    status = cpt212b_write_profile_to_flash(i2cspm, bytes,
                                            CPT212B_FLASH_PAYLOADS);
    if (status != SL_STATUS_OK)
      return status;

    sl_sleeptimer_delay_millisecond(5);
  }

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Write configuration profile to flash
 ******************************************************************************/
sl_status_t cpt212b_write_profile_to_flash(sl_i2cspm_t *i2cspm, uint8_t data[],
                                           uint16_t length)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];
  uint8_t                    i2c_write_data[1 + length];

  i2c_write_data[0] = CPT212B_CONF_WRITE;
  memcpy(&i2c_write_data[1], &data[0], sizeof(uint8_t) * length);

  seq.addr        = CPT212B_CONF_MODE_ADDR;
  seq.flags       = I2C_FLAG_WRITE;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1 + CPT212B_FLASH_PAYLOADS;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)
    return SL_STATUS_TRANSMIT;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Write CRC value to flash
 ******************************************************************************/
sl_status_t cpt212b_write_crc_to_flash(sl_i2cspm_t *i2cspm, uint8_t crc_1, 
                                       uint8_t crc_2)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[3];

  i2c_write_data[0] = CPT212B_CONF_WRITE_CRC;
  i2c_write_data[1] = crc_1;
  i2c_write_data[2] = crc_2;

  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 3;
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)
    return SL_STATUS_TRANSMIT;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Validate configuration profile from flash
 ******************************************************************************/
sl_status_t cpt212b_validate_configuration_profile(sl_i2cspm_t *i2cspm)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[1];

  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 1;

  seq.addr        = CPT212B_CONF_MODE_ADDR;
  seq.flags       = I2C_FLAG_READ;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone) 
    return SL_STATUS_TRANSMIT;  

  if (i2c_read_data[0] == CPT212B_CONF_PROFILE_VALID)
    return SL_STATUS_OK;
  
  return SL_STATUS_FAIL;
}

/***************************************************************************//**
 *   Enter sensing mode from configuration loading mode
 ******************************************************************************/
sl_status_t cpt212b_enter_sensing_mode(sl_i2cspm_t *i2cspm)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[2];

  i2c_write_data[0] = CPT212B_CONF_MODE_SELECT;
  i2c_write_data[1] = 0x01;
  seq.addr          = CPT212B_CONF_MODE_ADDR;
  seq.flags         = I2C_FLAG_WRITE;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  
  /* don't require to resend a new start address to read data */
  seq.buf[1].data   = i2c_read_data;
  seq.buf[1].len    = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)
    return SL_STATUS_TRANSMIT;

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Read keypad Interrupt event
 ******************************************************************************/
sl_status_t cpt212b_read_keypad_event(sl_i2cspm_t *i2cspm,
                                      uint8_t data[], uint16_t length)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];

  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 3;

  seq.addr        = CPT212B_SENSE_MODE_ADDR;
  seq.flags       = I2C_FLAG_READ;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2cspm, &seq);

  if (ret != i2cTransferDone)
    return SL_STATUS_TRANSMIT;

  // copy I2C data into return data array
  memcpy(data, i2c_read_data, sizeof(uint8_t)*length);

  return SL_STATUS_OK;
}

/***************************************************************************//**
 *   Reset the door key from user
 ******************************************************************************/
void cpt212b_reset_door_access(void)
{
  memset(keypadValue, 0, sizeof(uint8_t)*DOOR_KEY_ACCESS_LEN);
  keypadNum = 0;
}
