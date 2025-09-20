#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "parameters.h"

struct DipSwitches
{
  // When this is disabled, no dimming will occur.
  // The acceptance chirp is also disabled.
  // The set thresh indicator is slower? or just turn led off then comes on at
  // the end?
  bool enable_led_pwm;

  // When disabled, the LED will stay on.
  // When enabled, the LED will warn, then turn off after (n) hours and lock out
  // the mcu. This feature is primarily to protect the LED from being on for too
  // long. Will require a manual button press to resume normal ranging led
  // control. For repeat timeout violations the mcu shall shutdown?
  bool enable_led_shutoff_timeout; // TODO

  // When enabled, the controller shall switch to manual mode if an object is
  // held within the threshold distance for more than (n) seconds.
  // Alternatively, the # of samples needed and debounce time shall increase to
  // add delay to the hand in/out reset. Some future defined button sequence
  // shall reset to normal. The controller shall resume normal control when the
  // object is removed after a set delay.
  bool enable_grocery_detection; // TODO
};

/* Read the values of the DIP switches. Will temporarily enable pullups while
 * reading pin, then pull pins low before the function exits.*/
bool
read_dip_values(DipSwitches* jcf)
{
  // Enabled switches pull pin low.
  const int enable_lvl = 0;

  // Enable pullups on pins for reading.
  gpio_config_t gcf = {};
  gcf.mode = GPIO_MODE_INPUT;
  gcf.pull_up_en = GPIO_PULLUP_ENABLE;
  gcf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gcf.pin_bit_mask = (1ULL << DIP_ENABLE_LED_PWM_PIN) &
                     (1ULL << DIP_ENABLE_GROCERY_DETECT_PIN) &
                     (1ULL << DIP_ENABLE_LED_SHUTOFF_TIMEOUT_PIN);
  ESP_ERROR_CHECK(gpio_config(&gcf));

  // Let voltage on pins settle.
  vTaskDelay(1);

  jcf->enable_led_pwm = gpio_get_level(DIP_ENABLE_LED_PWM_PIN) & enable_lvl;
  jcf->enable_led_shutoff_timeout =
    gpio_get_level(DIP_ENABLE_LED_SHUTOFF_TIMEOUT_PIN) & enable_lvl;
  jcf->enable_grocery_detection =
    gpio_get_level(DIP_ENABLE_GROCERY_DETECT_PIN) & enable_lvl;

  // Disable pullups. Pull pins low.
  gcf.mode = GPIO_MODE_OUTPUT;
  gcf.pull_up_en = GPIO_PULLUP_DISABLE;
  gcf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&gcf));

  return true;
}