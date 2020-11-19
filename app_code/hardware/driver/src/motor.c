/***************************************************************************//**
 * @file
 * @brief Driver for the control DC motor
 ******************************************************************************/

#include "sl_sleeptimer.h"
#include "sl_app_assert.h"
#include "sl_sleeptimer.h"
#include "sl_app_log.h"
#include "sl_status.h"
#include "sl_pwm.h"
#include "motor.h"
#include "em_gpio.h"


sl_pwm_instance_t motor_pwm_instance = {
  .timer = MOTOR_PWM_PERIPHERAL,
  .channel = (uint8_t)(MOTOR_PWM_OUTPUT_CHANNEL),
  .port = (uint8_t)(MOTOR_CONTROL_PWM_PORT),
  .pin = (uint8_t)(MOTOR_CONTROL_PWM_PIN),
};

sl_pwm_config_t motor_pwm_config = {
  .frequency = MOTOR_PWM_FREQUENCY,
  .polarity = MOTOR_PWM_POLARITY,
};

uint8_t pwm_soft_start_profile[] = {
  25, 30, 40, 50, 60, 70, 80, 90, 100,
};

static volatile bool motor_control_enable;
static volatile bool motor_fault;

static sl_sleeptimer_timer_handle_t motor_driver_timer;
static void motor_driver_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data);

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/***************************************************************************//**
 *   Initialize GPIO module
 ******************************************************************************/
void motor_gpio_init(void)
{
  // Analog current output proportional to load current
  // GPIO_PinModeSet(MOTOR_CURRENT_IPROPI_PORT, MOTOR_CURRENT_IPROPI_PIN,
  //                 gpioModeInputPull, 1);

  // Fault indicator output from motor driver
  GPIO_PinModeSet(MOTOR_OUTPUT_nFAULT_PORT, MOTOR_OUTPUT_nFAULT_PIN,
                  gpioModeInputPullFilter, 1);

}

/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t motor_control_init(void)
{
  motor_control_enable = false;

  motor_gpio_init();

  sl_status_t sc = motor_pwm_init(true);
  sl_app_assert(sc == SL_STATUS_OK,
              "[E: 0x%04x] Failed to init dc motor.\n", (int)sc);
              
  return sc;
}

/***************************************************************************//**
 *    Enable the motor driver
 ******************************************************************************/
void motor_driver_enable(bool bEnable)
{
  if (bEnable == true)
  {
    GPIO_PinModeSet(MOTOR_ENABLE_SLEEP_PORT, MOTOR_ENABLE_SLEEP_PIN,
                    gpioModePushPull, 1);

    motor_control_enable = true;
  }
  else
  {
    GPIO_PinModeSet(MOTOR_ENABLE_SLEEP_PORT, MOTOR_ENABLE_SLEEP_PIN,
                    gpioModePushPull, 0);

    motor_control_enable = false;
  }
}

/***************************************************************************//**
 *    Enable the motor driver
 ******************************************************************************/
void motor_direction_control(bool bClockWise)
{
  if (bClockWise == true)
  {
    GPIO_PinModeSet(MOTOR_CONTROL_PH_PORT, MOTOR_CONTROL_PH_PIN,
                    gpioModePushPull, 1);
  }
  else
  {
    GPIO_PinModeSet(MOTOR_CONTROL_PH_PORT, MOTOR_CONTROL_PH_PIN,
                    gpioModePushPull, 0);
  }
}

/***************************************************************************//**
 *    initialize motor PWM module
 ******************************************************************************/
sl_status_t motor_pwm_init(bool bInit)
{
  sl_status_t sc = SL_STATUS_OK;

  if (bInit)
  {  
    sc = sl_pwm_init(&motor_pwm_instance, &motor_pwm_config);
  }
  else
  {
    sc = sl_pwm_deinit(&motor_pwm_instance);
    motor_driver_enable(false);
  }

  return sc;
}

/***************************************************************************//**
 *    motor execute lock operation
 ******************************************************************************/
sl_status_t motor_exec_lock(bool bEnableLock)
{
  sl_status_t sc;

  if ((motor_fault = motor_fault_indicator_read()) == true)
  {
    sl_app_log("motor driver return fault signal. \n");
    return SL_STATUS_ABORT;
  }

  if (bEnableLock == true)
  {
    sl_app_log("door lock.\n");
    motor_direction_control(true);
  }
  else
  {
     sl_app_log("door unlock.\n");
     motor_direction_control(false);
  }

  if (motor_control_enable != true)
    motor_driver_enable(true);

  motor_pwm_init(true);

  for(uint8_t i = 0; i < sizeof(pwm_soft_start_profile); i++)
  {
    sl_pwm_set_duty_cycle(&motor_pwm_instance, pwm_soft_start_profile[i]);

    if (i == 0)
      sl_pwm_start(&motor_pwm_instance);

    sl_sleeptimer_delay_millisecond(5);
  }

  // once DC motor soft-start is completed, allow DC motor to continue
  // running with 100% duty cycle
  // TODO[LAI]: optimize the time frame for reduce current consumption purpose
  sc = sl_sleeptimer_start_timer_ms(&motor_driver_timer, MOTOR_CONTROL_INTERVAL_MS,
                                    motor_driver_timer_cb, NULL, 0, 0);
                            
  return sc;
}

/***************************************************************************//**
 *    read the motor fault value
 ******************************************************************************/
bool motor_fault_indicator_read(void)
{
  return (GPIO_PinInGet(MOTOR_OUTPUT_nFAULT_PORT, MOTOR_OUTPUT_nFAULT_PIN) == 0);
}

/***************************************************************************//**
 *   motor driver callback to stop motor operation
 ******************************************************************************/
static void motor_driver_timer_cb(sl_sleeptimer_timer_handle_t *timer, void *data)
{
  (void)data;
  (void)timer;

  sl_pwm_stop(&motor_pwm_instance);
  motor_pwm_init(false);
}
