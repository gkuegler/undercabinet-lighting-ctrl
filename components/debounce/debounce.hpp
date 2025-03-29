/**
 * Debouncing routines taken and adapted from Jack Ganssle in his blog 'A Guide
 * to Debouncing, or, How to Debounce a Contact in Two Easy Pages, by Jack
 * Ganssle'. https://www.ganssle.com/debouncing.htm
 */

#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <esp_attr.h>

#include "sdkconfig.h"

#include <cstdint>
#include <iostream>

void
print_binary(const char* tag, const char* name, uint8_t value)
{
  char bits[9];
  for (int i = 7; i >= 0; --i) {
    bits[7 - i] = ((value >> i) & 1) ? '1' : '0';
  }
  bits[8] = '\0';
  ESP_LOGD(tag, "%s: %s", name, bits);
}

#define NOT_USED 0

// The sample buffer uses left shifts. At the buffer type to an unsigned integer
// with a width large enough to accomodate the number of desired samples.
// Compiler explorer on Xtensa ESP32-S3 gcc 12.2.0 showed unsigned 32 bit ints
// to generate the least instructions.
typedef uint8_t sample_buf_t;

// Clicks start at '1' so that the number returned from my sample function is
// the number of clicks.
enum class ButtonEvent
{
  LongPress = -1,
  None = 0,
  Click1 = 1,
  Click2,
  Click3,
  Click4,
};

class DebouncedButton
{
private:
  const gpio_num_t pin;
  // Future: make this a template specialization for pulldown buttons?
  // const bool pullup;

  // Future: use this for template specified debounce params
  // int sample_count;

  const char* tag = "DBNC";
  sample_buf_t sample_buf = 0;
  TickType_t pressed_ticks = 0;
  int click_count = 0;
  int state = 0;

public:
  bool enable_long_press = true;
  TickType_t long_press_delay = pdMS_TO_TICKS(2000);
  TickType_t initial_multi_click_delay = pdMS_TO_TICKS(300);
  TickType_t subsequent_multi_click_delay = pdMS_TO_TICKS(1000);

  // Specify a value greater than 0 to imediately return when this # of button
  // presses is reached. If '0' is specified, there is no limit.
  int multi_click_max = 0;

  DebouncedButton(gpio_num_t pin)
    : pin(pin) {};

  ButtonEvent sample(TickType_t ticks)
  {
    const sample_buf_t mask = 0b1111;
    const sample_buf_t rising_edge = 0b0111;
    const sample_buf_t falling_edge = 0b1000;

    sample_buf = ((sample_buf << 1) | gpio_get_level(pin)) & mask;

    // print_binary("bin", "name", rising_edge);
    // print_binary("bin", "name", falling_edge);
    // print_binary("dbnc", "sample", sample_buf);

    bool is_falling_edge = (sample_buf == falling_edge);
    bool is_rising_edge = (sample_buf == rising_edge);

    // Add leeway after the 2nd click.
    auto mclick_delay = (click_count < 2) ? initial_multi_click_delay
                                          : subsequent_multi_click_delay;

    switch (state) {
      case 0:
        click_count = 0;

        if (is_falling_edge) {
          click_count++;
          pressed_ticks = ticks;
          state = 1;
          // ESP_LOGD(tag, "1st btn down");
          return ButtonEvent::None;
        }
        break;

      case 1:
        if (enable_long_press && ticks - pressed_ticks > long_press_delay) {
          // ESP_LOGD(tag, "evt: long press");
          state = 0;
          return ButtonEvent::LongPress;
        }
        if (is_rising_edge) {
          state = 2;
        }
        break; // This could be removed and flow right into state 2?

      case 2:
        if (multi_click_max && click_count >= multi_click_max) {
          state = 0;
          // ESP_LOGD(tag, "Max multi-click reached");
          return static_cast<ButtonEvent>(click_count);
        }
        if (is_falling_edge) {
          pressed_ticks = ticks;
          click_count++;
          // ESP_LOGD(tag, "click %d", click_count);
          state = 3;
          return ButtonEvent::None;
        }

        if (ticks - pressed_ticks > mclick_delay) {
          // ESP_LOGD(tag, "evt: returning %d clicks", click_count);
          state = 0;
          return static_cast<ButtonEvent>(click_count);
        }
        break;

      case 3:
        if (is_rising_edge) {
          // ESP_LOGD(tag, "state -> 2");
          state = 2;
          return ButtonEvent::None;
        }
        break;
    }

    return ButtonEvent::None;
  }
};
