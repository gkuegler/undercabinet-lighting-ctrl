#include "rotary-encoder-polling.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "encoder";

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t *edata,
                                    void *pvParameter) {

  re_polling_rotary_encoder_t *encoder =
      (re_polling_rotary_encoder_t *)pvParameter;
  encoder->previous_pulse_count = 0;
  if (encoder->pcnt_unit) {
    pcnt_unit_clear_count(encoder->pcnt_unit);
  }

  // Don't return to a higher priority task.
  return false;
}

// Pulses may be missed during rollover if high and low limit or set above or
// below the maximum and minimum values of a 16-bit integer.
bool rep_initialize(re_polling_rotary_encoder_t *this) {
  // High and low limit of the internal pulse value register.
  // I use the full 16-bit register provided by ESP32 and handle the user
  // counter/position in a software check.
  const int high_limit = INT16_MAX;
  const int low_limit = INT16_MIN;

  ESP_LOGI(TAG, "Initializing counter.");

  pcnt_unit_config_t unit_config = {
      .high_limit = high_limit, .low_limit = low_limit, .flags.accum_count = 1};
  pcnt_unit_handle_t pcnt_unit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = this->min_pulse_duration_ns,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

  /*
   * Quadrature rotary encoder typical signal:
   * Note that this diagram and some data sheets have the convention the signal
   * will be pulled low when 'ON'.
   *
   * A      +-----+     +-----+     +-----+
   *          OFF |     |     |     |
   *              |  ON |     |     |
   *              +-----+     +-----+
   * B         +-----+     +-----+     +-----+
   *                 |     |     |     |
   *                 |     |     |     |
   *                 +-----+     +-----+
   *
   *  +--------------------------------------->
   *                 CW direction
   */

  // level_gpio_num is the control signal
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = this->pinA,
      .level_gpio_num = this->pinB,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

  // When pin A gets pulled low (while B is high), increase the value.
  // When pin A gets pulled low (while B is low), decrease the value.
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // When pin B gets pulled low (while A is high), decrease the value.
  // When pin B gets pulled low (while A is low), increase the value.
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = this->pinB,
      .level_gpio_num = this->pinA,
  };

  pcnt_channel_handle_t pcnt_chan_b = NULL;

  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  // Trigger a callback before counter rolls over.
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, high_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, low_limit));

  pcnt_event_callbacks_t cbs = {
      .on_reach = pcnt_on_reach,
  };

  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, this));
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

  this->pcnt_unit = pcnt_unit;
  this->value = 0;
  this->previous_pulse_count = 0;

  return true;
}

void rep_reset(re_polling_rotary_encoder_t *this) {
  // Add lock here if using a multithreaded environment
  this->previous_pulse_count = 0;
  this->value = 0;
  this->changed = true;
  ESP_ERROR_CHECK(pcnt_unit_clear_count(this->pcnt_unit));
}

re_count_t rep_get_value(re_polling_rotary_encoder_t *this) {
  return this->value;
}

re_count_t rep_get_delta(re_polling_rotary_encoder_t *this) {

  return this->delta;
}

/**
 * Returns true if the value was changed since it was last checked.
 */
bool rep_sample(re_polling_rotary_encoder_t *this) {
  int pulse_count = 0;

  ESP_ERROR_CHECK(pcnt_unit_get_count(this->pcnt_unit, &pulse_count));

  // Number of state changes, with direction.
  int delta = (pulse_count - this->previous_pulse_count) / 4;

  // Only change the value for a full quadrature signal, i.e 4 pulses
  // The ESP32S3 has an abs machine instruction. Nice!
  if (abs(delta) > 0) {
    this->previous_pulse_count += delta * 4;
    this->delta = delta;

    // Prevent the value from going over our max or under our min.
    const bool overflow = (this->value >= this->max) && delta > 0;
    const bool underflow = (this->value <= this->min) && delta < 0;

    if (!(overflow) && !(underflow)) {
      this->value += delta;
    }

    return true;
  }

  // This is to handle other threads calling 'set_value' in between samples.
  // In this case there would be no delta between the previous pulse count and
  // the current pulse count to indicate a change.
  if (this->changed) {
    this->changed = false;
    return true;
  }

  return false;
}