/**
 * Control of HC-SR04.
 */

#include "ultrasonic.hpp"

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_attr.h"     // using IRAM_ATTR
#include "esp_bit_defs.h" // using BIT64() macro
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "rom/ets_sys.h"     // using ets_delay_us()
#include "soc/gpio_struct.h" // using GPIO.out1_w1ts

#define GPIO_SET_FAST_0_31(pin)                                                \
  GPIO.out_w1ts = ((uint32_t)1 << (static_cast<int>(pin)))

#define GPIO_CLEAR_FAST_0_31(pin)                                              \
  GPIO.out_w1tc = ((uint32_t)1 << (static_cast<int>(pin)))

const char* HCSR04::TAG = "HCSR04";

/*
Measure pulse width with interrupt based flipflop.

The time it takes to handle the interrupt is usally about 2us.
This creates an effective debounce filter on the rising/falling edge for the
amount of time it takes for the interrupt to be handled and cleared.
Interrupts get cleared just before the IRQ returns.
*/
void IRAM_ATTR
HCSR04::echo_interrupt_handler(void* pvParameter)
{
  auto self = reinterpret_cast<HCSR04*>(pvParameter);

  if (!self->_pulse_capture) {
    self->_pulse_start_us = esp_timer_get_time();
    gpio_set_intr_type(self->_echo_pin, GPIO_INTR_LOW_LEVEL);
    self->_pulse_capture = true;
  } else {
    self->_pulse_duration_us = esp_timer_get_time() - self->_pulse_start_us;
    gpio_set_intr_type(self->_echo_pin, GPIO_INTR_HIGH_LEVEL);
    self->_pulse_in_flight = false;
    self->_pulse_capture = false;
  }

  // OPTIMIZATION: Implement inter-period triggering. Current Problem: if I
  // sample every 10ms, but the echo pulse completes at the 11ms mark I'd have
  // to wait another 9ms to trigger another ranging session. This reduces my
  // effective sampling rate to ~0.5x in certain situations.
  // Solution: I could use a task to continuously sample and store range in an
  // atomic variable, queue, or window?
  return;
}

void
HCSR04::init(gpio_num_t trig_pin, gpio_num_t echo_pin, int sample_period_ms)
{
  _trig_pin = trig_pin;
  _echo_pin = echo_pin;
  _ranging_timeout_start_count = 100 / sample_period_ms; // 100ms

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

float
HCSR04::sample()
{
  // TODO: Use ticks?
  if (_ranging_timeout_samples_remaining <= 0) {
    ESP_LOGE(
      TAG, "no echo pulse recived in %dms", _ranging_timeout_start_count * 10);
    _pulse_in_flight = false;
  }
  // Send a 10 microsecond pulse to the trigger pin.
  // Offloading pulse to peripherals is not warranted. Pulse is too short.
  if (!_pulse_in_flight) {
    // ESP_LOGD(TAG, "Starting Ranging Session");
    _pulse_in_flight = true;
    // Reset timeout.
    _ranging_timeout_samples_remaining = _ranging_timeout_start_count;
    // Disables all non-maskable interupts for accurate delay. Delay is so short
    // that there is no benefit moving the pusle to a peripheral.
    portDISABLE_INTERRUPTS();
    GPIO_SET_FAST_0_31(_trig_pin);
    ets_delay_us(10); // esp32 lowest level c api for delay
    GPIO_CLEAR_FAST_0_31(_trig_pin);
    portENABLE_INTERRUPTS();
  } else {
    --_ranging_timeout_samples_remaining;
  }
  return _pulse_duration_us * 0.01715;
}
