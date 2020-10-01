/***************************************************************************//**
 * @file
 * @brief Driver for the Battery level measurement
 * sensor
 ******************************************************************************/

#ifndef BATTERY_H_
#define BATTERY_H_

#include "sl_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
* @name Battery module Defines
* @{
*******************************************************************************/
#define CELL_BATTERY_MIN                    2.5
#define CELL_BATTERY_MAX                    3.3
#define MOTOR_BATTERY_MIN                   4.6
#define MOTOR_BATTERY_MAX                   6.5
#define COIL_VOLT_DIV_SCALE_FACT            CELL_BATTERY_MAX / 1.2
#define MOTOR_VOL_DIV_SCALE_FACT            MOTOR_BATTERY_MAX / 1.2

#define BATTERY_LEVEL_LOW                   25
#define BATTERY_LEVEL_MID                   50
#define BATTERY_LEVEL_HIGH                  75

#define NUM_IADC_SAMPLE                     200
#define NUM_IADC_INPUT                      2

#define BATTERY_MEASUREMENT_INTERVAL_SEC    1
#define BATTERY_MEASUREMENT_TOTAL_SEC       10
#define BATTERY_MOTOR_PROFILE_INTERVAL_MS   10
#define BATTERY_MOTOR_PROFILE_SAMPLE_SIZE   200

#define HFRCODPLL_FREQ                      cmuHFRCODPLLFreq_80M0Hz
#define CLK_SRC_ADC_FREQ                    40000000
#define CLK_ADC_FREQ                        10000000

// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_BUS                    CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC               GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS                    CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC               GPIO_CDBUSALLOC_CDODD0_ADC0

typedef enum
{
  CELL_BAT = 0,
  MOTOR_BAT = 1,
  MOTOR_PROFILE_BAT = 2,
  ALL_BAT = 99,
} battery_measure_mode_t;


/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t battery_measure_init(bool bEnableTest);

/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void battery_gpio_init(void);

/***************************************************************************//**
 *   Initialize IADC module to enable battery measurement
 ******************************************************************************/
void battery_IADC_init(void);

/***************************************************************************//**
 *   trigger IADC module for battery measurement
 ******************************************************************************/
sl_status_t battery_trigger_IADC_measure(battery_measure_mode_t measure_mode);

/***************************************************************************//**
 *   terminate IADC module for battery measurement
 ******************************************************************************/
void battery_terminate_IADC_Measure(battery_measure_mode_t measure_mode);

/***************************************************************************//**
 *   trigger IADC module to scan again for battery measurement
 ******************************************************************************/
void battery_trigger_IADC_scan_again(void);

/***************************************************************************//**
 *   IADC Handler
 ******************************************************************************/
void IADC_IRQHandler(void);

/***************************************************************************//**
 *   send the first indication for battery level 
 ******************************************************************************/
void battery_send_first_indication (void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H_ */
