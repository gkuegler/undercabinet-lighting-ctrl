#include <stdbool.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

#include "inttypes.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*** Local component includes ***/
#include "debounce.h"

/*** Local includes ***/
#include "filter.hpp"
#include "led.hpp"
#include "rotary-encoder-polling.h"
#include "ultrasonic.hpp"

#define GUI_PROCESSOR_CORE_ID 1
#define CONFIG_LV_DEF_REFR_PERIOD 5

static const char *TAG = "[main]";

/*** USER INPUT DEVICES ***/
static re_polling_rotary_encoder_t *encoder;
static HCSR04 hcsr04;
static Filter<float, 10.0f, 5, 4, 30> filter;
static Led light;

/*** OTHERS ***/
QueueHandle_t event_q;

// Creates a semaphore to handle concurrent call to lvgl stuff.
// If you wish to call *any* lvgl function from other threads/tasks
// you should lock on the very same semaphore.
static SemaphoreHandle_t xGuiSemaphore;

/*** STATIC PROTOTYPES ***/
static void initialize_controls();
static void guiLoop(void *pvParameter);
static void button_callback(void *);

extern "C" void app_main(void) {
  light.init(GPIO_NUM_2);

  // Run GUI on core 1.
  xTaskCreatePinnedToCore(guiLoop, "gui-loop", 4096 * 2, NULL, 3, NULL,
                          GUI_PROCESSOR_CORE_ID);

  // For debugging purposes.
  esp_intr_dump(NULL);

  return;
}

void initialize_controls() {
  db_edge_input_t button;
  button.cb = button_callback;
  button.pin = 14;
  button.etype = DB_EDGE_FALLING;
  button.pull_up = DB_PIN_PULLUP;
  button.sample_count = 8;
  button.sample_period_ms = 5;
  button.sampling_task_priority = 5;
  button.cb_task_stack_size = 4096 * 2;
  button.core_id = 1;
  db_register_edge(&button);

  encoder = (re_polling_rotary_encoder_t *)malloc(
      sizeof(re_polling_rotary_encoder_t));

  if (NULL == encoder) {
    ESP_LOGD(TAG, "Failed to allocate encoder memory.");
    abort();
  }

  encoder->max = 63;
  encoder->min = 0;
  encoder->step_size = 1;
  encoder->overflow = true;
  encoder->pinA = 18;
  encoder->pinB = 16;
  encoder->min_pulse_duration_ns = CONFIG_RE_DEFAULT_MIN_PULSE_DURATION_NS;

  if (!rep_initialize(encoder)) {
    abort();
  }
}

uint32_t get_milliseconds() { return esp_timer_get_time() / 1000; }

static void sample_inputs() {
  if (rep_sample(encoder)) {
    light.set_user_duty(encoder->value);
  }

  float d = hcsr04.sample();
  filter.filter_sample(d);

  // TODO: use lvgl style queue and handle where I only update once per cycle?
  light.update_timeout_tick();

  // Display distance for testing.
  auto dist = static_cast<unsigned int>(d);
  ESP_LOGI(TAG, "d: %0.2f", d);
}

static void handle_events() {
  Event event;
  while (pdTRUE == xQueueReceive(event_q, &event,
                                 0 // Return immediately if empty.
                                 )) {
    switch (event) {
    case Event::EVENT_HAND_ENTER:
      ESP_LOGI(TAG, "Toggle the light.");
      light.toggle();
      break;
    case Event::EVENT_HAND_EXIT:
      break;
    }
  }
}

static void guiLoop(void *pvParameter) {
  (void)pvParameter; // not used

  event_q = create_event_q();
  initialize_controls();
  hcsr04.init(GPIO_NUM_9, GPIO_NUM_10);
  filter.init(event_q);

  while (1) {
    // I use my own timing here because its easier to control which core this
    // loop runs on with 'xTaskCreatePinnedToCore' and monitor how long my
    // loop takes for performance testing.
    const int64_t start = esp_timer_get_time();

    sample_inputs();

    handle_events();

    // Suspend the main loop until the next.
    const int64_t task_duration_us = (esp_timer_get_time() - start);
    ESP_LOGV(TAG, "gui loop delta us: %" PRIu64, task_duration_us);

    if (task_duration_us <= CONFIG_LV_DEF_REFR_PERIOD * 1000) {
      vTaskDelay(
          pdMS_TO_TICKS(CONFIG_LV_DEF_REFR_PERIOD - (task_duration_us / 1000)));
    } else {
      ESP_LOGW(TAG, "gui loop duration exceded refresh period: %" PRIu64 "us",
               task_duration_us);
    }
  }

  vTaskDelete(NULL);
}
static void button_callback(void *) { rep_reset(encoder); }