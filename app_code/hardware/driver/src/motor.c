/***************************************************************************//**
 * @file
 * @brief Control DC motor via pwm module
 ******************************************************************************/

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
  // GPIO_PinModeSet(MOTOR_OUTPUT_nFAULT_PORT, MOTOR_OUTPUT_nFAULT_PIN,
  //                 gpioModeInputPullFilter, 1);

}

/***************************************************************************//**
 *    Initialize battery measurement via IADC module
 ******************************************************************************/
sl_status_t motor_control_init(void)
{
  motor_gpio_init();

  motor_control_enable = false;  

  sl_status_t sc = motor_pwm_init(true);

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
 *    execute door lock operation
 ******************************************************************************/
void door_lock_exec(bool bEnableLock)
{
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
    {
      sl_pwm_start(&motor_pwm_instance);
    }

    sl_sleeptimer_delay_millisecond(5);
  }

  sl_sleeptimer_delay_millisecond(1000);

  sl_pwm_stop(&motor_pwm_instance);
  motor_pwm_init(false);
}
