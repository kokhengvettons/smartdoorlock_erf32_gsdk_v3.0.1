/***************************************************************************//**
 * @file
 * @brief Battery measure via IADC module
 ******************************************************************************/

#include "sl_sleeptimer.h"
#include "sl_app_assert.h"
#include "sl_app_log.h"
#include "sl_status.h"
#include "sl_bt_api.h"
#include "battery.h"
#include "em_iadc.h"
#include "em_cmu.h"
#include "string.h"
#include "gatt_db.h"
#include "app.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

#define FEATURE_MEASURE_MOTOR_BATTERY_PROFILE           0

static battery_measure_mode_t battery_measure_mode;
static sl_sleeptimer_timer_handle_t battery_motor_profile_timer;
static sl_sleeptimer_timer_handle_t battery_timer;

static uint32_t battery_timer_counter;
static uint16_t IadcSteps[NUM_IADC_INPUT];
static uint16_t IadcStepsMotor[NUM_IADC_SAMPLE];
static uint16_t Iadc_sample_idx;
static uint8_t battery_level[NUM_IADC_INPUT];

static volatile bool battery_level_enable;
static volatile bool battery_test_enable;

// Periodic timer callback.
static void battery_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data);
static void battery_motor_profile_timer_cb(sl_sleeptimer_timer_handle_t *timer,
                                           void *data);

/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t battery_measure_init(bool bEnableTest)
{
  if (battery_level_enable == true)
    return SL_STATUS_OK;

  battery_test_enable = bEnableTest;

  battery_gpio_init();

  battery_IADC_init();

  return battery_trigger_IADC_measure(ALL_BAT);
}

/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void battery_gpio_init(void)
{
  GPIO_PinModeSet(gpioPortC, 0x00, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(gpioPortC, 0x06, gpioModeInputPullFilter, 1);
}

/***************************************************************************//**
 *   Initialize IADC module to enable battery measurement
 ******************************************************************************/
void battery_IADC_init(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT;

  // Enable IADC0 clock branch
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified by
  // other code
  IADC_reset(IADC0);

  // Set HFRCODPLL band and the tuning value based on the value in the
  // calibration table made during production.
  CMU_HFRCODPLLBandSet(HFRCODPLL_FREQ);

  // Select HFRCODPLL as the EM01GRPA clock
  CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFRCODPLL);

  // Select clock for IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM01GRPACLK);

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // 25ns per cycle, 40000 cycles make 1ms timer event
  init.timerCycles = 20000; // 0.5ms

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                             CLK_ADC_FREQ,
                                             0,
                                             iadcCfgModeNormal,
                                             init.srcClkPrescale);

  // Scan initialization
  initScan.triggerSelect = iadcTriggerSelTimer;
  initScan.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID2;

  // Tag FIFO entry with scan table entry id.
  initScan.showId = true;

  // Configure entries in scan table, CH0 is single ended from input 0, CH1 is
  // single ended from input 1
  // PC00 -> P01 on BRD4001 J102
  initScanTable.entries[0].posInput = iadcPosInputPortCPin0;
  initScanTable.entries[0].negInput = iadcNegInputGnd;
  initScanTable.entries[0].includeInScan = true;

  // PC06 -> P29 on BRD4001 J102
  initScanTable.entries[1].posInput = iadcPosInputPortCPin6;
  initScanTable.entries[1].negInput = iadcNegInputGnd;
  initScanTable.entries[1].includeInScan = true;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

  // Enable the IADC timer - can only be done after the IADC has been enabled
  IADC_command(IADC0, iadcCmdEnableTimer);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;

  // Enable Scan interrupts
  IADC_enableInt(IADC0, IADC_IEN_SCANFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/***************************************************************************//**
 *   trigger IADC module for battery measurement
 ******************************************************************************/
sl_status_t battery_trigger_IADC_measure(battery_measure_mode_t measure_mode)
{
  sl_status_t sc = SL_STATUS_OK;
  battery_measure_mode = measure_mode;
  battery_timer_counter = 0;

  if ((measure_mode == MOTOR_PROFILE_BAT) &&
      (FEATURE_MEASURE_MOTOR_BATTERY_PROFILE == 1))
  {
    Iadc_sample_idx = 0;
    memset(IadcStepsMotor, 0, sizeof(IadcStepsMotor));

    sc = sl_sleeptimer_start_periodic_timer_ms(&battery_motor_profile_timer,
                                               BATTERY_MOTOR_PROFILE_INTERVAL_MS,
                                               battery_motor_profile_timer_cb,
                                               NULL, 0, 0);
  }
  else
  {
    sc = sl_sleeptimer_start_periodic_timer_ms(&battery_timer,
                                               BATTERY_MEASUREMENT_INTERVAL_SEC * 1000,
                                               battery_timer_cb, NULL, 0, 0);
  }

  battery_trigger_IADC_scan_again();

  // send first indication
  battery_send_first_indication();

  // set battery level enable flag to true
  battery_level_enable = true;
  sl_app_log("Battery Level: enable battery level measurement \n");
  return sc;
}

/***************************************************************************//**
 *   terminate IADC module for battery measurement
 ******************************************************************************/
void battery_terminate_IADC_Measure(battery_measure_mode_t measure_mode)
{
  // exit if battery level measurement doesn't enable before
  if (battery_level_enable != true)
    return;

  if (measure_mode == MOTOR_PROFILE_BAT)
  {
     (void)sl_sleeptimer_stop_timer(&battery_motor_profile_timer);
    IADC_command(IADC0, iadcCmdStopScan);

    sl_app_log("Battery Level: disable battery motor profile measurement \n");
  }
  else
  {
    // terminate all battery level measurement
    (void)sl_sleeptimer_stop_timer(&battery_timer);
    IADC_command(IADC0, iadcCmdStopScan);
    battery_level_enable = false;

    sl_app_log("Battery Level: disable battery level measurement \n");
  }
}

/***************************************************************************//**
 *   trigger IADC module to scan again for battery measurement
 ******************************************************************************/
void battery_trigger_IADC_scan_again(void)
{
  // reset IadcSteps only when restart the IADC Scan
  memset(IadcSteps, 0, sizeof(IadcSteps));

  IADC_command(IADC0, iadcCmdStartScan);
}

/***************************************************************************//**
 *   IADC Handler
 ******************************************************************************/
void IADC_IRQHandler(void)
{
  IADC_Result_t sample;

  // Get ADC results
  while(IADC_getScanFifoCnt(IADC0))
  {
    // Read data from the scan FIFO
    sample = IADC_pullScanFifoResult(IADC0);

    // collect door lock/unlock motor battery voltage profile
    if ((battery_measure_mode == MOTOR_PROFILE_BAT) &&
        (FEATURE_MEASURE_MOTOR_BATTERY_PROFILE == 1))
    {
      // motor battery profile using same pin layout as motor battery
      if (sample.id == MOTOR_BAT)
      {
        IadcStepsMotor[Iadc_sample_idx++] = sample.data;
      }
    }

    IadcSteps[sample.id] = sample.data;
  }

  // Start next IADC conversion
  IADC_clearInt(IADC0, IADC_IF_SCANFIFODVL);
}

/***************************************************************************//**
 *   Timer callback for battery measurement
 ******************************************************************************/
static void battery_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;

  battery_timer_counter++;

  if (battery_timer_counter == BATTERY_MEASUREMENT_TOTAL_SEC - 2)
  {
      battery_trigger_IADC_scan_again();
  }
  else if (battery_timer_counter == BATTERY_MEASUREMENT_TOTAL_SEC - 1)
  {
    float volt_cell;
    float volt_motor;
    float batteryPercentage; 

    // calculate battery % for cell coil battery
    volt_cell = (IadcSteps[CELL_BAT] * 3.3 / 0xFFF) * 1;
    batteryPercentage = (volt_cell - CELL_BATTERY_MIN) /
        (CELL_BATTERY_MAX - CELL_BATTERY_MIN) * 100;
    batteryPercentage = batteryPercentage > 100 ? 100 : batteryPercentage;
    battery_level[CELL_BAT] = (uint8_t) batteryPercentage;

    // calculate battery % for dc motor battery
    volt_motor = (IadcSteps[MOTOR_BAT] * 3.3 / 0xFFF) * 1;
    batteryPercentage = (volt_motor - MOTOR_BATTERY_MIN) /
        (MOTOR_BATTERY_MAX - MOTOR_BATTERY_MIN) * 100;
    batteryPercentage = batteryPercentage > 100 ? 100 : batteryPercentage;
    battery_level[MOTOR_BAT] = (uint8_t) batteryPercentage;

    // update attributes value
    sl_bt_gatt_server_write_attribute_value(gattdb_battery_level_cell, 0, 1,
                                            &battery_level[CELL_BAT]);
    sl_bt_gatt_server_write_attribute_value(gattdb_battery_level_motor, 0, 1,
                                            &battery_level[MOTOR_BAT]);
    
    sl_app_log("Battery Level: cell bat - %d%% \n", battery_level[CELL_BAT]);
    sl_app_log("Battery Level: motor bat - %d%% \n", battery_level[MOTOR_BAT]);

    // validate battery result to confirm battery circuit working fine.
    if (battery_test_enable == true)
    {
      battery_test_enable = false;
      uint8_t err_code = special_cmd_success;

      if ((volt_cell <= 0 && volt_cell >= 3.6) ||
          (volt_motor <= 0 && volt_motor >= 6.5))
      {
        err_code = special_cmd_err_hardware_battery;
      }

      uint16_t len = 0;
      sl_status_t sc = sl_bt_gatt_server_send_characteristic_notification(
          0xFF, gattdb_special_command, 1, &err_code, &len);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to send char notification. \n",
                    (int)sc);

      battery_terminate_IADC_Measure(ALL_BAT);

      return;
    }

    if (battery_level[CELL_BAT] < BATTERY_LEVEL_LOW)
    {
      uint16_t len = 0;
      sl_bt_gatt_server_send_characteristic_notification(
          0xFF, gattdb_battery_level_cell, 1,
          &battery_level[CELL_BAT], &len);
      
      sl_app_log("Battery Level: cell battery is level low:  %d%%\n",
                 battery_level[CELL_BAT]);
    }
  }
  else if (battery_timer_counter >= BATTERY_MEASUREMENT_TOTAL_SEC)
  {
    battery_timer_counter = 0;

    // schedule 1 seconds delay to prevent two notifications send at same time
    if (battery_level[MOTOR_BAT] < BATTERY_LEVEL_LOW)
    {
      uint16_t len = 0;
      sl_bt_gatt_server_send_characteristic_notification(
          0xFF, gattdb_battery_level_motor, 1,
          &battery_level[MOTOR_BAT], &len);

      sl_app_log("Battery Level: motor battery level is low: %d%%\n",
            battery_level[MOTOR_BAT]);
    }
  }
}

/***************************************************************************//**
 *   Timer callback for battery motor profile measurement
 ******************************************************************************/
static void battery_motor_profile_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;

  if (Iadc_sample_idx >= BATTERY_MOTOR_PROFILE_SAMPLE_SIZE )
  {
    (void)sl_sleeptimer_stop_timer(&battery_motor_profile_timer);
    Iadc_sample_idx = 0;
  }
  else
  {
    battery_trigger_IADC_scan_again();
  }  
}

/***************************************************************************//**
 *   send the first indication for battery level 
 ******************************************************************************/
void battery_send_first_indication (void)
{
  if (battery_level_enable != true)
  {
    // immediately trigger to measure the battery level
    battery_timer_counter = BATTERY_MEASUREMENT_TOTAL_SEC - 3;
    battery_timer_cb(&battery_timer, NULL);
  }
}
