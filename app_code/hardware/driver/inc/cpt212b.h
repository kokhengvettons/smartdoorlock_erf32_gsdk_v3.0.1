/***************************************************************************//**
 * @file
 * @brief Driver for the CPT212B Capacitive Sense keypad
 * sensor
 ******************************************************************************/

#ifndef CPT212B_H_
#define CPT212B_H_

#include <stdbool.h>
#include "sl_status.h"
#include "cpt212b_a01_gm_init.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
* @name CPT212B module Defines
* @{
*******************************************************************************/
  /** I2C sense mode events type for cpt212b */
#define CPT212B_SENSE_MODE_ADDR         0xE0
#define CPT212B_CONF_MODE_ADDR          0xC0

/** I2C sense mode events type for cpt212b */
#define CPT212B_SENSE_EVENT_TOUCH       0x00
#define CPT212B_SENSE_EVENT_RELEASE     0x01
#define CPT212B_SENSE_EVENT_PROXIMITY   0x03

/** I2C configuration loading mode for cpt212b */
#define CPT212B_CONF_MODE_SELECT        0x08
#define CPT212B_CONF_UNLOCK             0x09
#define CPT212B_CONF_ERASE              0x0A
#define CPT212B_CONF_WRITE              0x0B
#define CPT212B_CONF_WRITE_CRC          0x0C

/** I2C touch events for cpt212b */
#define CPT212B_TOUCH_EVENT_CS0         0x00
#define CPT212B_TOUCH_EVENT_CS1         0x01
#define CPT212B_TOUCH_EVENT_CS2         0x02
#define CPT212B_TOUCH_EVENT_CS3         0x03
#define CPT212B_TOUCH_EVENT_CS4         0x04
#define CPT212B_TOUCH_EVENT_CS5         0x05
#define CPT212B_TOUCH_EVENT_CS6         0x06
#define CPT212B_TOUCH_EVENT_CS7         0x07
#define CPT212B_TOUCH_EVENT_CS8         0x08
#define CPT212B_TOUCH_EVENT_CS9         0x09
#define CPT212B_TOUCH_EVENT_CS10        0x0A
#define CPT212B_TOUCH_EVENT_CS11        0x0B

/** I2C configuration profile validity check results for cpt212b */
#define CPT212B_CONF_PROFILE_VALID      0x80
#define CPT212B_CONF_PROFILE_INVALID    0x01

/** I2C Hardware for cpt212b **/
#define CPT212B_I2CSENSOR_ENABLE_PIN    0x01
#define CPT212B_I2CSENSOR_RESET_PIN     0x03
#define CPT212B_I2CSENSOR_CONTROL_PORT  gpioPortC

/** configuration profile mode */
#define CPT212B_FLASH_PAYLOADS          0x08
#define CPT212B_MAX_WRITE_CONF_LEN      512

/** door access key */
#define DOOR_KEY_ACCESS_LEN             6

/** @}  */

/***************************************************************************//**
 *   Initialize the cpt212b keypad.
 ******************************************************************************/
sl_status_t cpt212b_init(sl_i2cspm_t *i2cspm, bool bEnableFlashNewPRofile);

/***************************************************************************//**
 *   Initialize GPIO to enable the CPT212B Capacitive touch keypad
 ******************************************************************************/
void cpt212b_gpio_init(void);

/***************************************************************************//**
 *   Enable CPT212B Capacitive touch keypad Interrupt
 ******************************************************************************/
void cpt212b_interrupt_enable(void);

/***************************************************************************//**
 *   Enable I2C module (active low) for CPT212B Capacitive touch keypad
 ******************************************************************************/
void cpt212b_I2C_enable(bool bEnable);

/***************************************************************************//**
 *   Flash a new configuration profile into CPT212B Capacitive touch keypad
 ******************************************************************************/
sl_status_t cpt212b_flash_new_profile(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   Enter configuration loading unlock sequence
 ******************************************************************************/
sl_status_t cpt212b_loading_unlock_seqence(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   Erase the configuration profile
 ******************************************************************************/
sl_status_t cpt212b_erase_configuration_profile(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   Write configuration profile into non-volatile memory
 ******************************************************************************/
sl_status_t cpt212b_flash_configuration_profile(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   write configuration profile
 ******************************************************************************/
sl_status_t cpt212b_write_profile_to_flash(sl_i2cspm_t *i2cspm, uint8_t data[],
                                  uint16_t length);


/***************************************************************************//**
 *   Write CRC value to flash
 ******************************************************************************/
sl_status_t cpt212b_write_crc_to_flash(sl_i2cspm_t *i2cspm, uint8_t crc_1, 
                                       uint8_t crc_2);

/***************************************************************************//**
 *   Validate configuration profile from flash
 ******************************************************************************/
sl_status_t cpt212b_validate_configuration_profile(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   Enter sensing mode from configuration loading mode
 ******************************************************************************/
sl_status_t cpt212b_enter_sensing_mode(sl_i2cspm_t *i2cspm);

/***************************************************************************//**
 *   Read keypad Interrupt event
 ******************************************************************************/
sl_status_t cpt212b_read_keypad_event(sl_i2cspm_t *i2cspm, 
                                      uint8_t data[], uint16_t length);

/***************************************************************************//**
 *   Reset the door key from user
 ******************************************************************************/
void cpt212b_reset_door_access(void);

#ifdef __cplusplus
}
#endif

#endif /* CPT212B_H_ */
