#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "driver/ledc.h"
#include "esp_attr.h" // for IRAM_ATTR
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class Led {
private:
  enum class STATE { OFF, ON };
  static constexpr const char *TAG = "LEDC";

public:
  ledc_channel_t channel = LEDC_CHANNEL_0;
  ledc_timer_t timer = LEDC_TIMER_0;
  //   The S3 doesn't have the distinction between low and high speed anymore;
  //   that distinction in general was about having hardware to latch the
  //   settings when the PWM ended anyway, it didn't have anything to do with
  //   the max speed you can get out of a channel.
  ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer_bit_t res = LEDC_TIMER_6_BIT;
  uint16_t user_duty = pow(2, LEDC_TIMER_6_BIT) - 1; // set max duty 100%
  uint16_t cur_duty = user_duty;                     // current led duty
  STATE state = STATE::ON;
  int hpoint = 0; // duty cycle wave start period
  uint32_t warning_level = 0;

  float warning_dim_frac = 0.20;
  int warning_dim_lvl = static_cast<int>(warning_dim_frac * user_duty);
  int warn_timeout = 20 * (1000 / 10);
  int shutoff_timeout = 10 * (1000 / 10);
  int count = 0;

  Led(){};
  ~Led(){};

  void init(gpio_num_t pin, int sample_period_ms, int timeout1_m,
            int timeout2_m) {
    warn_timeout = (timeout1_m * 60 * 1000) / sample_period_ms;
    shutoff_timeout = (timeout2_m * 60 * 1000) / sample_period_ms;
    // clang-format off
    /*
    LEDC_CLKx           | PWM Hz | Highest Resolution (bit) 1 | Lowest Resolution (bit) 2
    APB_CLK (80 MHz)    |  1 kHz | 16 | 7
    APB_CLK (80 MHz)    |  5 kHz | 13 | 4
    APB_CLK (80 MHz)    | 10 kHz | 12 | 3
    RC_FAST_CLK (8 MHz) |  1 kHz | 12 | 3
    RC_FAST_CLK (8 MHz) |  2 kHz | 11 | 2
    REF_TICK (1 MHz)    |  1 kHz |  9 | 1
    */
    // clang-format on

    // https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#ledpwm
    uint32_t bits =
        ledc_find_suitable_duty_resolution(80 * 1000 * 1000, 40 * 1000);
    ESP_LOGI("LEDC",
             "maximum possible duty resolution in bits for "
             "ledc_timer_config: %" PRIu32,
             bits);

    ledc_timer_config_t timercfg;
    timercfg.speed_mode = this->speed_mode;
    timercfg.duty_resolution = res;
    timercfg.timer_num = this->timer;
    timercfg.freq_hz = 40000;
    timercfg.clk_cfg = LEDC_USE_APB_CLK;
    timercfg.deconfigure = 0; // don't deconfigure exg timer
    ESP_ERROR_CHECK(ledc_timer_config(&timercfg));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t chancfg;
    chancfg.speed_mode = this->speed_mode;
    chancfg.channel = this->channel;
    chancfg.timer_sel = this->timer;
    chancfg.intr_type = LEDC_INTR_DISABLE;
    chancfg.gpio_num = pin;
    chancfg.duty = this->cur_duty;
    chancfg.hpoint = this->hpoint;
    ESP_ERROR_CHECK(ledc_channel_config(&chancfg));
  }

  void set_user_duty(uint16_t d) {
    user_duty = d;
    set_duty(d);
  }

  void set_duty(uint16_t d) {
    // This API call is thread safe.
    // This API call needs a fade service installed on the channel before use.
    // ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, d,
    // hpoint));
    cur_duty = d;
    ESP_ERROR_CHECK(ledc_set_duty(speed_mode, channel, cur_duty));
    ESP_ERROR_CHECK(ledc_update_duty(speed_mode, channel));
  }

  /* Set LED dim level based on current state.*/
  void update() {
    if (STATE::ON == state) {
      // Restore the current active duty.
      if (1 == warning_level) {
        set_duty(warning_dim_lvl);
      } else if (2 == warning_level) {
        set_duty(0);
      } else {
        set_duty(user_duty);
      }
    } else {
      // Turn the light off.
      set_duty(0);
    }
  }

  void update_timeout_tick() {
    if (state == STATE::OFF) {
      return;
    }

    ++count;
    if (warning_level < 2 && count >= shutoff_timeout + warn_timeout) {
      warning_level = 2;
      update();
    } else if (warning_level < 1 && count >= warn_timeout) {
      warning_level = 1;
      update();
    }
  }

  void reset_timeout() {}

  void enter_timeout_warning() {
    // dim to 50% of cur dim lvl?
  }

  void toggle() {
    if (STATE::ON == state) {
      if (warning_level > 0) { // User renewed lights.
        count = 0;
        warning_level = 0;
      } else { // User turned off lights.
        state = STATE::OFF;
      }
    } else { // User turned on lights.
      state = STATE::ON;
      warning_level = 0;
      count = 0;
    }
    update();
  }
};
