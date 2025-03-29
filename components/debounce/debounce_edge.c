/**
 * Debouncing routines taken and adapted from Jack Ganssle in his blog 'A Guide
 * to Debouncing, or, How to Debounce a Contact in Two Easy Pages, by Jack
 * Ganssle'. https://www.ganssle.com/debouncing.htm
 */

#include "debounce.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <esp_attr.h>

#include "sdkconfig.h"

#define NOT_USED 0

#define DIGITAL_READ(x) gpio_get_level(x)

static const char* TAG = "dbnce";

// The sample buffer uses left shifts. At the buffer type to an unsigned integer
// with a width large enough to accomodate the number of desired samples.
// Compiler explorer on Xtensa ESP32-S3 gcc 12.2.0 showed unsigned 32 bit ints
// to generate the least instructions.
typedef uint32_t sample_buf_t;

typedef bool (*sampling_func_t)(db_edge_input_t*);

typedef struct
{
  db_edge_input_t* input;
  sampling_func_t sample;
} db_edge_input_ex_t;

/**
 * Poll a pin for effective falling edge detection.
 *
 * Example if CONFIG_DB_DEFAULT_SAMPLE_COUNT = 12 and 16-bit state buffer.
 * Here's a diagram of what value the 'state' variable will hold to
 * determine when a falling edge has happened.
 *
 *  ignored bits     last bouncy 'one'     stable pin state 'low'
 *         \                |                     /
 *          \__________     |        ____________/
 *                      \   |       /
 *                     |--| | |---------|
 *      state -->      1111 1 00000000000
 *
 * When the GPI pin is high, 1's will be shifted onto the state variable.
 * When a user presses the button to pull the GPIO low, a mixture of 1's & 0's
 * will propagate through the stream as the signal bounces. At some point
 * there's the last bounce (a one) followed by a stream of zeroes. As the button
 * remains depressed, state continues to propagate zeroes. There's just the one
 * time, when the last bouncy "one" was in the upper bit position, that the code
 * returns a TRUE.
 * We OR in 1's in the upper bits to ignore everything but the number of lower
 * bids equal to the sample count. * *
 */
static bool
debounced_is_falling_edge(db_edge_input_t* input)
{
  const sample_buf_t ignore =
    (sample_buf_t)((sample_buf_t)-1 << (input->sample_count + 1));
  const sample_buf_t fall_edge =
    (sample_buf_t)((sample_buf_t)-1 << (input->sample_count));

  static sample_buf_t state = 0;
  state = ignore | (state << 1) | !DIGITAL_READ(input->pin);
  if (state == fall_edge) {
    return true;
  }
  return false;
}

static bool
debounced_is_rising_edge(db_edge_input_t* input)
{
  const sample_buf_t ignore = ((sample_buf_t)-1 << (input->sample_count + 1));
  const sample_buf_t rise_edge =
    ((sample_buf_t)-1 >>
     ((sizeof(sample_buf_t) * 8) - (input->sample_count + 1)));

  static sample_buf_t state = 0; // Current debounce status
  state = ignore | (state << 1) | !DIGITAL_READ(input->pin);
  if (state == rise_edge) {
    return true;
  }
  return false;
}

// bool debounced_digital_read() {
//   /* GCC >12 will optimize these constants out at at a minimum of
//   optimizations enabled. Verified on: -O1, -O2, -O3 and -Og.*/ const
//   sample_buf_t empty_buf =
//       (sample_buf_t)((sample_buf_t)-1 << (CONFIG_DB_DEFAULT_SAMPLE_COUNT +
//       1));
//   const sample_buf_t state_high_buf = -1;

//   static sample_buf_t debounced_state = 0;
//   static sample_buf_t candidate_state = 0;
//   candidate_state = empty_buf | (candidate_state << 1) | !digital_pin_read();
//   if (candidate_state == state_high_buf) {
//     debounced_state = 1;
//   } else if (candidate_state == empty_buf) {
//     debounced_state = 0;
//   }
//   return debounced_state;
// }

static void
sampling_task(void* pvParameters)
{

  db_edge_input_ex_t* config = (db_edge_input_ex_t*)pvParameters;
  db_edge_input_t* input = config->input;
  sampling_func_t sample = config->sample;

  ESP_LOGD(TAG, "sample ptr: %p", sample);

  // Initialise the xLastWakeTime variable with the current time.
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(input->sample_period_ms));

    // Perform action if state change detected.
    if (sample(input)) {
      input->cb(input->cb_params);
    }
  }
}

static void
register_gpio(int pin, pin_pull_t pullup)
{
  gpio_config_t io_conf = {};

  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << pin);

  if (pullup == DB_PIN_PULLUP) {
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
  } else if (pullup == DB_PIN_PULLDOWN) {
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
  } else {
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
  }

  // configure GPIO with the given settings
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

bool
db_register_edge(db_edge_input_t* input)
{
  if (NULL == input) {
    ESP_LOGE(TAG, "NULL check failled on 'input'.");
  }
  if (!input->cb) {
    ESP_LOGE(TAG, "No callback function pointer provided.");
  }
  if (!input->pin) {
    ESP_LOGE(TAG, "No pin provided.");
  }

  db_edge_input_t* internal = (db_edge_input_t*)malloc(sizeof(db_edge_input_t));

  db_edge_input_ex_t* config =
    (db_edge_input_ex_t*)malloc(sizeof(db_edge_input_ex_t));

  if (internal && input && config) {
    memcpy((void*)internal, (void*)input, sizeof(db_edge_input_t));
    config->input = internal;
  }

  // Will abort on a failure in debug modes.
  register_gpio(input->pin, input->pull_up);

  ESP_LOGD(TAG, "falling edge ptr: %p", &debounced_is_falling_edge);

  // Set the proper sampling function.
  if (DB_EDGE_FALLING == input->etype) {
    ESP_LOGD(TAG, "set falling edge");
    config->sample = &debounced_is_falling_edge;
  } else if (DB_EDGE_RISING == input->etype) {
    config->sample = &debounced_is_rising_edge;
  }

  char task_name[configMAX_TASK_NAME_LEN];
  sprintf(task_name, "edge_smpl_pin%d", input->pin);

  // DEPRECATED: this function is deprecated in favor of more modern core
  // affinities
  xTaskCreatePinnedToCore(sampling_task,
                          task_name,
                          input->cb_task_stack_size,
                          (void*)config,
                          input->sampling_task_priority,
                          NULL,
                          input->core_id);

  return true;
}
