/**
 * Control of HC-SR04.
 */

#include "ultrasonic.hpp"

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_attr.h"     // using IRAM_ATTR
#include "esp_bit_defs.h" // using BIT64() macro
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"     // using ets_delay_us()
#include "soc/gpio_struct.h" // using GPIO.out1_w1ts

#define GPIO_SET_FAST_0_31(pin)                                                \
  GPIO.out_w1ts = ((uint32_t)1 << (static_cast<int>(pin)))

#define GPIO_CLEAR_FAST_0_31(pin)                                              \
  GPIO.out_w1tc = ((uint32_t)1 << (static_cast<int>(pin)))

/*
Measure pulse width with interrupt based flipflop.

The time it takes to handle the interrupt is usally about 2us.
This creates an effective debounce filter on the rising/falling edge for the
amount of time it takes for the interrupt to be handled and cleared.
Interrupts get cleared just before the IRQ returns.
*/
void IRAM_ATTR HCSR04::echo_interrupt_handler(void *pvParameter) {
  auto sensor = reinterpret_cast<HCSR04 *>(pvParameter);
  if (!sensor->_pulse_in_flight) {
    sensor->_pulse_in_flight = true;
    sensor->_pulse_start_us = esp_timer_get_time();
    gpio_set_intr_type(sensor->_echo_pin, GPIO_INTR_LOW_LEVEL);
  } else {
    sensor->_pulse_duration_us = esp_timer_get_time() - sensor->_pulse_start_us;
    gpio_set_intr_type(sensor->_echo_pin, GPIO_INTR_HIGH_LEVEL);
    sensor->_pulse_in_flight = false;
  }

  // TODO: I could use a queue to stash measurements and wake up a task to
  // immediatly take a new measurement if its been longer than my sampling
  // period? Current Problem: if I sample every 10ms, but this completes at
  // the 11ms mark I'd have to wait another 9ms to trigger another pulse.
  // This reduces my effective sampling rate to ~2x in certain situations.
  // This would introduce a 9ms lag when going from long distances to short
  // distances.
  return;
}

void HCSR04::init(gpio_num_t trig_pin, gpio_num_t echo_pin) {
  // Note: Using external 10K pulldown resistors on the trigger and echo pins.
  _trig_pin = trig_pin;
  _echo_pin = echo_pin;

  // TRIGGER PIN
  gpio_config_t trcfg = {};
  trcfg.intr_type = GPIO_INTR_DISABLE;
  trcfg.mode = GPIO_MODE_OUTPUT;
  trcfg.pin_bit_mask = BIT64(static_cast<int>(_trig_pin));
  trcfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  trcfg.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&trcfg));

  // ECHO PIN
  gpio_config_t eccfg = {};
  eccfg.intr_type = GPIO_INTR_HIGH_LEVEL;
  eccfg.mode = GPIO_MODE_INPUT;
  eccfg.pin_bit_mask = BIT64(static_cast<int>(_echo_pin));
  eccfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  eccfg.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&eccfg));

  // Install interrupt on echo pin. An interrupt based flipflop is used to
  // measure pulse widths. Using interrupt priority #3 because this is the
  // highest available user-level interrupt that can be allocated with C
  // handlers.
  ESP_ERROR_CHECK(gpio_install_isr_service(
      ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3));
  ESP_ERROR_CHECK(
      gpio_isr_handler_add(_echo_pin, echo_interrupt_handler, this));
}

float HCSR04::sample() {
  // Send a 10 microsecond pulse to the trigger pin.
  // Offloading pulse to peripherals is not warranted. Pulse is too short and
  // does not need to be exact.
  if (!_pulse_in_flight) {
    portDISABLE_INTERRUPTS(); // Disables all non-maskable interupts.
    GPIO_SET_FAST_0_31(_trig_pin);
    ets_delay_us(10); // lowest level c api for delay
    GPIO_CLEAR_FAST_0_31(_trig_pin);
    portENABLE_INTERRUPTS();
  }
  return _pulse_duration_us * 0.01715;
}
