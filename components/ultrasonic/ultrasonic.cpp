/**
 * Control of HC-SR04.
 */

#define LOGPOINT ESP_LOGE("BRK", "%s: %d", __FILE__, __LINE__);

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

// ESP-IDF fastest way to set a gpio pin level.
#define GPIO_SET_FAST_0_31(pin)                                                \
  GPIO.out_w1ts = ((uint32_t)1 << (static_cast<int>(pin)))

#define GPIO_CLEAR_FAST_0_31(pin)                                              \
  GPIO.out_w1tc = ((uint32_t)1 << (static_cast<int>(pin)))

const char* HCSR04::tag = "HCSR04";

/*
Measure pulse width with a flip/flop.
Note: the speed of sound at sea level is ~30us per cm.
Interupt handlers take ~2us to enter, but this delay is cancelled out becuase if
occurs on both the start and finish times of the pulse.
*/
void IRAM_ATTR
HCSR04::echo_interrupt_handler(void* pvParam)
{
  static BaseType_t xHigherPriorityTaskWoken = 0;
  static const char* tag = "ISR-ECHO";
  static uint64_t estart = 0;
  static uint32_t etime = 0;
  static bool ecapture = false;

  auto self = reinterpret_cast<HCSR04*>(pvParam);

  // Ignore if no pulse is in flight. Must be noise on pif.
  if (!self->_pulse_in_flight) {
    ESP_EARLY_LOGE(tag, "No pulse in flight.");
    return;
  }

  // Manually flip/flop intr levels because ESP32 can't do rising/falling
  // interupts on shared interrupts.
  if (!ecapture) {
    estart = esp_timer_get_time();
    ecapture = true;
    gpio_set_intr_type(self->_echo_pin, GPIO_INTR_LOW_LEVEL);
  } else {
    // Truncate to utilize atomics. 32bits of microseconds give a max time of
    // ~72 minutes. That's well above expected echo pulse durations.
    etime = static_cast<uint32_t>(esp_timer_get_time() - estart);
    ecapture = false;
    gpio_set_intr_type(self->_echo_pin, GPIO_INTR_HIGH_LEVEL);

    self->_pulse_in_flight = 0;

    // Wake up task to handle ranging result. This should set
    // xHigherPriorityTaskWoken to pdTRUE, thus imediately enabling a context
    // switch into my ranging handler task.
    if (self->_task_handle) {
      xTaskNotifyIndexedFromISR(self->_task_handle,
                                self->_task_idx,
                                etime,
                                eSetValueWithOverwrite,
                                &xHigherPriorityTaskWoken);
    }

    // If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    // should be performed to ensure the interrupt returns directly to the
    // highest priority task.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void
HCSR04::init(gpio_num_t trig_pin,
             gpio_num_t echo_pin,
             TaskHandle_t task_to_notify,
             BaseType_t task_idx)
{
  _trig_pin = trig_pin;
  _echo_pin = echo_pin;
  _task_handle = task_to_notify;

  // TRIGGER SEND PIN
  gpio_config_t trcfg = {};
  trcfg.pin_bit_mask = BIT64(static_cast<int>(_trig_pin));
  trcfg.mode = GPIO_MODE_OUTPUT;
  trcfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&trcfg));

  // ECHO RECEIVE PIN
  gpio_config_t eccfg = {};
  eccfg.pin_bit_mask = BIT64(static_cast<int>(_echo_pin));
  eccfg.mode = GPIO_MODE_INPUT;
  eccfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  eccfg.intr_type = GPIO_INTR_HIGH_LEVEL;
  ESP_ERROR_CHECK(gpio_config(&eccfg));

  // Install interrupt on echo pin. Opt for a shared interrupt to avoid
  // unecessarily consuming a hardware interrupt based flipflop is used to
  // measure pulse widths. Using interrupt priority (#3) because this is the
  // highest available user-level interrupt that can be allocated with C
  // handlers.
  ESP_ERROR_CHECK(gpio_install_isr_service(
    ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3));

  ESP_ERROR_CHECK(
    gpio_isr_handler_add(_echo_pin, echo_interrupt_handler, this));
}

void
HCSR04::trigger_ranging_session()
{
  // Disabled interupts for accurate delay.
  // Send a 10 microsecond pulse to the trigger pin.
  // Delay is too short to bother moving the pusle to a peripheral.
  taskENTER_CRITICAL(&mux);
  GPIO_SET_FAST_0_31(_trig_pin);
  ets_delay_us(10); // esp32 lowest level c api for delay
  GPIO_CLEAR_FAST_0_31(_trig_pin);
  taskEXIT_CRITICAL(&mux);
}

constexpr float
HCSR04::convert_us_to_range(const uint32_t us)
{
  // ESP32's can't do float math in an interupt handler.
  return us * 0.01715f;
}

void
HCSR04::reset()
{
  if (_timed_out_ranges++ > 5) {
    _timed_out_ranges = 0;
    _delay_between_ranging_ms++;

    ESP_LOGW(
      tag, "Increasing delay between ranges: %dms", _delay_between_ranging_ms);
  }

  _pulse_in_flight = false;
}

float
HCSR04::range_and_wait()
{
  while (true) {

    // Will return immediately if the specified time is in the past.
    vTaskDelayUntil(&_last_range_ticks, _delay_between_ranging_ms);

    // There should technically be no range in flight when we get here.
    // Only one ranging is allowed at a time.
    if (_pulse_in_flight <= 0) {
      _pulse_in_flight += 1;
      trigger_ranging_session();
    } else {
      vTaskDelay(1);
      continue;
    }

    uint32_t ulval = 0;

    // See notes in the ultrasonic component src files about the chip's
    // timeouts. The firmware has a hard timeout if the range is too long or
    // weak, but we add another timeout here, in case the signal is missed.
    if (xTaskNotifyWait(
          0, ULONG_MAX, &ulval, pdMS_TO_TICKS(_range_timeout_ms)) == pdFALSE) {
      ESP_LOGW(tag, "Ranging session watchdog timed out.");
      reset();
      continue;
    }

    _last_range_ticks = xTaskGetTickCount();

    // Ignore pulses less than 11us ~= 1mm range.
    if (ulval < _min_pulse_duration_us) {
      ESP_LOGW(tag, "Ignoring short pulse: %sus", ulval);
      continue;
    }

    return convert_us_to_range(ulval);
  }
}
