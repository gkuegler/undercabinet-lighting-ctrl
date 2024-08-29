/*
High-performance rotary encoder with callbacks.
 */

#include "rotary-encoder-interrupt.h"

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// TODO: not i need to use internal counters and constantly poll
//  i can add a callback and internally keep track of when different
// see accum param of pcnt
// TODO: optimize only one rotary encoder update task is required

// My way is overly complicated and setting the counter while someone is
// rotating it will cause missed values anyway.

static const char *TAG = "rotary";

static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t *edata,
                                    void *pvParameter) {

  re_interrupt_rotary_encoder_t *this =
      (re_interrupt_rotary_encoder_t *)pvParameter;

  BaseType_t xHigherPriorityTaskWoken = pdTRUE;

  re_count_t amt =
      (edata->watch_point_value / CONFIG_RE_DEFAULT_PULSES_PER_DETENT) *
      this->step_size;

  // Though built-in increment and decrement operators and 'compound
  // assignments' are read-modify-write atomic operations with total
  // sequentially consistent ordering acording in C11, I'm using this
  // concurrency support function for explicitness.
  atomic_fetch_add(&this->value, amt);

  // This will increment a counter by one but how do I decrement?
  xTaskNotifyFromISR(this->counter_handler_task, 0UL, eNoAction,
                     &xHigherPriorityTaskWoken);

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
  should be performed to ensure the interrupt returns directly to the highest
  priority task. The macro used for this purpose is dependent on the port in
  use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  return 0;
}

static void counter_gui_update_task(void *pvParameter) {
  re_interrupt_rotary_encoder_t *this =
      (re_interrupt_rotary_encoder_t *)pvParameter;

  for (;;) {
    // Wait to be notified of an interrupt.
    if (xTaskNotifyWait(
            pdFALSE,   // Don't clear bits on entry. Notified value not used.
            ULONG_MAX, // Clear all bits on exit. Notified value not used.
            NULL,      // Stores the notified value.
            pdMS_TO_TICKS(5000)) == pdTRUE) {

      re_count_t value = atomic_load(&this->value);

      this->stable_count = value;

      if (value > this->max) {
        value = this->overflow ? this->min : this->max;
        // Count could change to anything between load and store calls.
        // Global atomic value could change from 11 to 15.
        // Setting to zero would destroy that information.
        // I would need to do multiple buffering do not miss any increments.
        // I think it's okay if I miss counter changes at the max and min
        // boundaries.
        atomic_store(&this->value, value);
      } else if (value < this->min) {
        value = this->overflow ? this->max : this->min;
        atomic_store(&this->value, value);
      }

      ESP_LOGI(TAG, "rotary encoder value: %d", value);

      if (this->cb) {
        this->cb(value);
      }
    }
  }
}

re_interrupt_rotary_encoder_t *
re_interrupt_rotary_encoder_initialize(re_interrupt_rotary_encoder_t *this) {
  ESP_LOGI(TAG, "Initializing counter.");

  const int high_limit = CONFIG_RE_DEFAULT_PULSES_PER_DETENT;
  const int low_limit = -(CONFIG_RE_DEFAULT_PULSES_PER_DETENT);

  char task_name[configMAX_TASK_NAME_LEN];
  sprintf(task_name, "rotary-%d%d", this->pinA, this->pinB);
  ESP_LOGD(TAG, "task name: %s", task_name);

  if (pdPASS !=
      xTaskCreate(&counter_gui_update_task, task_name, this->cb_task_stack_size,
                  this, this->cb_task_priority, &this->counter_handler_task)) {
    ESP_LOGE(TAG, "Could not create rotary encoder task.");
    return NULL;
  }

  pcnt_unit_config_t unit_config = {
      .high_limit = high_limit,
      .low_limit = low_limit,
  };
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

  // Trigger a callback to occur whenever the knob is turned one detent to the
  // left or right.
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, low_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, high_limit));

  pcnt_event_callbacks_t cbs = {
      .on_reach = pcnt_on_reach,
  };

  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, this));
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

  this->pcnt_unit = pcnt_unit;

  return this;
}

void re_interrupt_encoder_set_value(re_interrupt_rotary_encoder_t *counter,
                                    re_count_t value, bool notify_cb) {
  atomic_store(&counter->value, value);
  if (counter->cb && notify_cb) {
    counter->cb(value);
  }
}