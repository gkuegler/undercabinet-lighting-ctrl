/**
 * Left shift buffer debouncing routines taken and adapted from Jack Ganssle in
 * his blog 'A Guide to Debouncing, or, How to Debounce a Contact in Two Easy
 * Pages, by Jack Ganssle'. https://www.ganssle.com/debouncing.htm
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Outputs the binary representation of the sample buffer, rissing & falling
// matches for every sample.
// #define TEST_LOG_DBNC_BUTTON_BINARY_BUF

// Clicks start at '1' so that the number returned from the sample method is
// the number of clicks.
enum ButtonEvent : int
{
  BUTTON_EVT_LONG_PRESS = -1,
  BUTTON_EVT_NONE = 0,
};

enum class GpioPullDirection
{
  None,
  Up,
  Down
};

class DebouncedButton
{
private:
  // The sampling left shifts digital reads onto the buf. Compiler explorer on
  // Xtensa ESP32-S3 gcc 12.2.0 showed uint32_t to generate the least
  // instructions.
  typedef uint32_t sample_buf_t;

  static constexpr const char* tag = "DbncBtn";
  sample_buf_t sample_buf = 0;
  TickType_t pressed_ticks = 0;
  int click_count = 0;
  int state = 0;

  const gpio_num_t pin;
  const GpioPullDirection pin_pull;

public:
  bool enable_long_press = true;
  TickType_t long_press_delay = pdMS_TO_TICKS(2000);
  TickType_t initial_multi_click_delay = pdMS_TO_TICKS(300);
  TickType_t subsequent_multi_click_delay = pdMS_TO_TICKS(1000);

  // Specify a value greater than 0 to imediately return when this # of button
  // presses is reached & sampled. If '0' is specified, there is no limit. The
  // subsequent_multi_click_delay will then need to occur to dermine when the
  // multi-click event has ended.
  int max_multi_clicks = 0;

  DebouncedButton(gpio_num_t pin, GpioPullDirection pin_pull)
    : pin(pin)
    , pin_pull(pin_pull) {};

  void init()
  {
    gpio_config_t g = {};
    g.mode = GPIO_MODE_INPUT;
    g.pin_bit_mask = (1ULL << (int)pin);

    if (pin_pull == GpioPullDirection::Up) {
      g.pull_up_en = GPIO_PULLUP_ENABLE;
    } else if (pin_pull == GpioPullDirection::Down) {
      g.pull_down_en = GPIO_PULLDOWN_ENABLE;
    }

    ESP_ERROR_CHECK(gpio_config(&g));
  }

  // Returns a button event as the type of click. See special case for a long
  // click.
  int sample(TickType_t ticks)
  {
    // where n is no. of mask bits
    const sample_buf_t mask = 0b1111;         // ((1 << n) - 1)
    const sample_buf_t rising_edge = 0b0111;  // ((1 << (n - 1)) - 1)
    const sample_buf_t falling_edge = 0b1000; // (1 << (n - 1))

    sample_buf = ((sample_buf << 1) | gpio_get_level(pin)) & mask;

#ifdef TEST_LOG_DBNC_BUTTON_BINARY_BUF
    // print_binary("bin", "rising", rising_edge);
    // print_binary("bin", "falling", falling_edge);
    // print_binary("bin", "sample_buf", sample_buf);
#endif // TEST_LOG_DBNC_BUTTON_BINARY_BUF

    bool is_falling_edge = (sample_buf == falling_edge);
    bool is_rising_edge = (sample_buf == rising_edge);

    // Add a bit longer acceptable delay after the 2nd click.
    auto mclick_delay = (click_count <= 1) ? initial_multi_click_delay
                                           : subsequent_multi_click_delay;

    // See the 'button state machine.drawio.png' file for a flowchart of the
    // state machine.
    switch (state) {
      case 0:
        click_count = 0;

        if (is_falling_edge) {
          click_count++;
          pressed_ticks = ticks;
          state = 1;
        }
        break;

      case 1:
        if (enable_long_press && ((ticks - pressed_ticks) > long_press_delay)) {
          state = 0;
          return BUTTON_EVT_LONG_PRESS;
        }
        if (is_rising_edge) {
          state = 2;
        }
        break;

      case 2:
        if (max_multi_clicks && (click_count >= max_multi_clicks)) {
          state = 0;
          return click_count;
        }

        if (is_falling_edge) {
          pressed_ticks = ticks;
          click_count++;
          state = 3;
          return BUTTON_EVT_NONE;
        }

        if ((ticks - pressed_ticks) > mclick_delay) {
          state = 0;
          return click_count;
        }
        break;

      case 3:
        if (is_rising_edge) {
          state = 2;
        }
        break;
    }

    return BUTTON_EVT_NONE;
  }

#ifdef TEST_LOG_DBNC_BUTTON_BINARY_BUF
private:
  static void print_binary(const char* tag, const char* name, uint8_t value)
  {
    char bits[9];
    for (int i = 7; i >= 0; --i) {
      bits[7 - i] = ((value >> i) & 1) ? '1' : '0';
    }
    bits[8] = '\0';
    ESP_LOGD(tag, "%s: %s", name, bits);
  }
#endif // TEST_LOG_DBNC_BUTTON_BINARY_BUF
};
