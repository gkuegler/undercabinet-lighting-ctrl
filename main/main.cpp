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
#include "event.hpp"
#include "filter.hpp"
#include "led.hpp"
#include "rotary-encoder-polling.h"
#include "ultrasonic.hpp"

#define HMI_PROCESSOR_CORE_ID 1
#define POLLING_PERIOD_MS 10 // ms

#define MODULE_ADAFRUIT_QTPY_ESP32_S3

#ifdef MODULE_ADAFRUIT_QTPY_ESP32_S3
#define CONFIG_DUTY_RELAY_PIN GPIO_NUM_18
#define CONFIG_ROTARY_BTN_PIN GPIO_NUM_17
#define CONFIG_ROTARY_A_PIN GPIO_NUM_8
#define CONFIG_ROTARY_B_PIN GPIO_NUM_9
#define CONFIG_TRIGGER_PIN GPIO_NUM_16
#define CONFIG_ECHO_PIN GPIO_NUM_6
#endif // MODULE_ADAFRUIT_QTPY_ESP32_S3

static const char *TAG = "[main]";

#define EVENT_QUEUE_COUNT 10
uint8_t ucQueueStorage[EVENT_QUEUE_COUNT * sizeof(Event)];
EventQueue event_q(EVENT_QUEUE_COUNT, ucQueueStorage);

/*** USER INPUT DEVICES ***/
// static re_polling_rotary_encoder_t *encoder;
static HCSR04 hcsr04;
static HandGestureFilter<float, 10.0f, 5, 4, 30> filter;
static Led led;

/*** STATIC PROTOTYPES ***/
static void initialize_controls();
static void hmi_loop(void *pvParameter);
// static void button_callback(void *);

extern "C" void app_main(void) {
  led.init(CONFIG_DUTY_RELAY_PIN, POLLING_PERIOD_MS, 60, 120);

  // Run GUI on core 1.
  xTaskCreatePinnedToCore(hmi_loop, "gui-loop", 4096 * 2, NULL, 3, NULL,
                          HMI_PROCESSOR_CORE_ID);

  // For debugging purposes.
  esp_intr_dump(NULL);

  return;
}

uint32_t get_milliseconds() { return esp_timer_get_time() / 1000; }

static void sample_inputs() {
  // if (rep_sample(encoder)) {
  //   led.set_user_duty(encoder->value);
  // }

  float d = hcsr04.sample();
  filter.filter_sample(d);

  led.update_timeout_tick();

  // Display distance for testing.
  // auto dist = static_cast<unsigned int>(d);
  // ESP_LOGI(TAG, "d: %0.2f", d);
}

static void handle_events() {
  Event event;
  while (true == event_q.receive(event)) {
    switch (event) {
    case Event::EVENT_HAND_ENTER:
      ESP_LOGI(TAG, "Toggle the led.");
      led.toggle();
      break;
    case Event::EVENT_HAND_EXIT:
      break;
    }
  }
}

static void hmi_loop(void *pvParameter) {
  (void)pvParameter; // not used

  event_q.init();
  filter.init(&event_q);

  initialize_controls();

  while (1) {
    // I use my own timing here because its easier to control which core this
    // loop runs on with 'xTaskCreatePinnedToCore' and monitor how long my
    // loop takes for performance testing.
    const int64_t start = esp_timer_get_time();

    sample_inputs();
    handle_events();

    // Suspend the main loop until the next iteration.
    const int64_t task_duration_us = (esp_timer_get_time() - start);
    ESP_LOGV(TAG, "gui loop delta us: %" PRIu64, task_duration_us);

    if (task_duration_us <= POLLING_PERIOD_MS * 1000) {
      vTaskDelay(pdMS_TO_TICKS(POLLING_PERIOD_MS - (task_duration_us / 1000)));
    } else {
      ESP_LOGW(TAG, "gui loop duration exceded refresh period: %" PRIu64 "us",
               task_duration_us);
    }
  }

  vTaskDelete(NULL);
}
void initialize_controls() {
  // db_edge_input_t button;
  // button.cb = button_callback;
  // button.pin = static_cast<int>(CONFIG_ROTARY_BTN_PIN);
  // button.etype = DB_EDGE_FALLING;
  // button.pull_up = DB_PIN_PULLUP;
  // button.sample_count = 8;
  // // TODO: change this to a class? Why do I specify sampling here?
  // button.sample_period_ms = 5;
  // button.sampling_task_priority = 5;
  // button.cb_task_stack_size = 4096 * 2;
  // button.core_id = 1;
  // db_register_edge(&button);

  // encoder = (re_polling_rotary_encoder_t *)malloc(
  //     sizeof(re_polling_rotary_encoder_t));

  // if (NULL == encoder) {
  //   ESP_LOGD(TAG, "Failed to allocate encoder memory.");
  //   abort();
  // }

  // encoder->max = 63;
  // encoder->min = 0;
  // encoder->step_size = 1;
  // encoder->overflow = true;
  // encoder->pinA = static_cast<int>(CONFIG_ROTARY_A_PIN);
  // encoder->pinB = static_cast<int>(CONFIG_ROTARY_B_PIN);
  // encoder->min_pulse_duration_ns = CONFIG_RE_DEFAULT_MIN_PULSE_DURATION_NS;

  // if (!rep_initialize(encoder)) {
  //   abort();
  // }

  hcsr04.init(CONFIG_TRIGGER_PIN, CONFIG_ECHO_PIN, POLLING_PERIOD_MS);
}

// static void button_callback(void *) { rep_reset(encoder); }