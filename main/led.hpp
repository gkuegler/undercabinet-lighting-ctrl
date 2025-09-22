#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "driver/ledc.h"
#include "esp_attr.h" // for IRAM_ATTR
#include "esp_clk_tree.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Local Inclues */
#include "parameters.h"
#include "util.h"

/*
Not Thread Safe
*/
class Led
{
private:
  static constexpr const char* tag = "LED";
  static constexpr ledc_timer_bit_t pwm_res_bits = LEDC_TIMER_6_BIT;
  ledc_channel_t channel = LEDC_CHANNEL_0;
  ledc_timer_t timer = LEDC_TIMER_0;
  /* The S3 doesn't have the distinction between low and high speed anymore;
  that distinction was about having hardware to latch the
  settings when the PWM ended, it didn't have anything to do with
  the max speed you can get out of a channel. */
  ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;
  uint32_t max_duty = 0;
  gpio_num_t outout_pin;
  TickType_t last_change_ticks;
  int safety_timeout_ms = HOURS_TO_MS(12);
  float user_bright_lvl = 1.0f;
  bool state = true;
  bool enabled = true;

public:
  bool pwm_enabled = true; // readonly
  bool safety_timeout_enabled = false;

  Led(gpio_num_t outout_pin)
    : outout_pin(outout_pin) {};
  ~Led() {};

  void init(bool pwm_enabled)
  {
    // clang-format off
    /*
    // https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#ledpwm
    --- TABLE 28.2-1 ---
    LEDC_CLKx           | PWM Hz | Highest Resolution (bit) 1 | Lowest Resolution (bit) 2
    -------------------------------------------------------------------------------------
    APB_CLK (80 MHz)    |  1 kHz |            16              |            7
    APB_CLK (80 MHz)    |  5 kHz |            13              |            4
    APB_CLK (80 MHz)    | 10 kHz |            12              |            3
    RC_FAST_CLK (8 MHz) |  1 kHz |            12              |            3
    RC_FAST_CLK (8 MHz) |  2 kHz |            11              |            2
    REF_TICK (1 MHz)    |  1 kHz |             9              |            1
    */
    // clang-format on

    // APB clock should be 80MHZ. It's freq can't dynamically vary so the 2nd
    // param is ignored.
    if (pwm_enabled) {
      uint32_t apb_clock_freq = 80 * 1000 * 1000;
      ESP_ERROR_CHECK(
        esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                     &apb_clock_freq));

      ESP_LOGI(tag, "APB Clock Frequency: %" PRIu32, apb_clock_freq);

      uint32_t pwm_res_bits =
        ledc_find_suitable_duty_resolution(apb_clock_freq, LED_PWM_FREQ_HZ);
      ESP_LOGI("LEDC",
               "maximum possible duty resolution in bits for "
               "ledc_timer_config: %" PRIu32,
               pwm_res_bits);

      // Integer that represents 100% duty.
      max_duty = pow(2, pwm_res_bits) - 1;

      ledc_timer_config_t timercfg;
      timercfg.speed_mode = this->speed_mode;
      timercfg.duty_resolution = (ledc_timer_bit_t)pwm_res_bits;
      timercfg.timer_num = this->timer;
      timercfg.freq_hz = LED_PWM_FREQ_HZ;
      timercfg.clk_cfg = LEDC_USE_APB_CLK;
      timercfg.deconfigure = 0; // don't deconfigure exg timer
      ESP_ERROR_CHECK(ledc_timer_config(&timercfg));

      // Prepare and then apply the LEDC PWM channel configuration.
      ledc_channel_config_t chancfg;
      chancfg.speed_mode = this->speed_mode;
      chancfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
      chancfg.channel = this->channel;
      chancfg.timer_sel = this->timer;
      chancfg.intr_type = LEDC_INTR_DISABLE;
      chancfg.gpio_num = outout_pin;
      chancfg.duty = (uint32_t)0;
      chancfg.hpoint = 0;
      ESP_ERROR_CHECK(ledc_channel_config(&chancfg));
    } else {
      gpio_config_t io_conf = {};
      io_conf.mode = GPIO_MODE_OUTPUT;
      io_conf.pin_bit_mask = (1ULL << (int)outout_pin);
      ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
  }

  /* Shuts down led if it has been on for more than the safety timeout. */
  void check_safety_timeout(TickType_t ticks)
  {
    if (safety_timeout_enabled && state &&
        ticks - last_change_ticks > pdMS_TO_TICKS(safety_timeout_ms)) {
      off();
    }
  }

  // Converts a float percentage (0.0 to 1.0) to a n-bit integer
  uint32_t percent_to_bit(float pc)
  {
    if (pc <= 0.0f) {
      return 0;
    } else if (pc > 1.0f) {
      return max_duty;
    } else {
      // +0.5 for rounding
      return static_cast<uint32_t>(pc * (float)max_duty + 0.5f);
    }
  }

  void set_brightness_user(float lvl)
  {
    // This API call needs a fade service installed on the channel before use.
    // ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, d,
    // hpoint));
    user_bright_lvl = lvl;
    state = (lvl > 0.0f);
    set_brightness(lvl);
  }

  // This API call is thread safe.
  void set_brightness(float lvl)
  {
    // This API call needs a fade service installed on the channel before use.
    // ESP_ERROR_CHECK(ledc_set_duty_and_update(speed_mode, channel, d,
    // hpoint));
    last_change_ticks = xTaskGetTickCount();

    if (pwm_enabled) {
      ESP_LOGI(tag, "brightness: %.2f", lvl);
      ESP_ERROR_CHECK(ledc_set_duty(speed_mode, channel, percent_to_bit(lvl)));
      ESP_ERROR_CHECK(ledc_update_duty(speed_mode, channel));
    } else {
      gpio_set_level(outout_pin, static_cast<int>(lvl > 0.0f));
    }
  }

  void resume()
  {
    if (state) {
      set_brightness(user_bright_lvl);
    } else {
      set_brightness(0.0f);
    }
  }

  void off()
  {
    state = false;
    set_brightness(0.0f);
  }

  void on()
  {
    state = true;
    set_brightness(user_bright_lvl);
  }

  void toggle()
  {
    if (state) {
      off();
    } else {
      on();
    }
  }

  void pulse(int count, int delayoff, int delayon)
  {
    // If lights are on, briefly turn off.
    if (state) {
      set_brightness(0.0f);
    }

    if (!pwm_enabled) {
      return;
    }

    for (int i = 0; i < count; i++) {
      vTaskDelay(pdMS_TO_TICKS(delayoff));
      set_brightness(1.0f);
      vTaskDelay(pdMS_TO_TICKS(delayon));
      set_brightness(0.0f);
    }
  }
};
