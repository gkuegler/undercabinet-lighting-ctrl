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
#include "lvgl.h"
#include "st7789.h"

/*** Local includes ***/
#include "filter.hpp"
#include "led.hpp"
#include "rotary-encoder-polling.h"
#include "ultrasonic.hpp"

#define GUI_PROCESSOR_CORE_ID 1

static const char *TAG = "[main]";

// Match the requested SPI transfer size with the desired buffer size.
// This sets how much of the screen can be redrawn at one time.
#define BUF_SIZE CONFIG_ST7789_MAX_TRANSACTION_SZ

/*** USER INPUT DEVICES ***/
static re_polling_rotary_encoder_t *encoder;
static HCSR04 hcsr04;
static Filter<float, 10.0f, 5, 4, 30> filter;
static Led light;

/*** LVGL STATIC OBJECTS ***/
static lv_obj_t *label;
static lv_display_t *disp;

/*** OTHERS ***/
QueueHandle_t event_q;

// Creates a semaphore to handle concurrent call to lvgl stuff.
// If you wish to call *any* lvgl function from other threads/tasks
// you should lock on the very same semaphore.
static SemaphoreHandle_t xGuiSemaphore;

/*** STATIC PROTOTYPES ***/
static void initialize_controls();
static void guiLoop(void *pvParameter);
static void flush_callback();
static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void create_demo_widgets(void);
static void button_callback(void *);

#ifdef __cplusplus
extern "C" void app_main(void) {
#else
void app_main(void) {
#endif

  xGuiSemaphore = xSemaphoreCreateMutex();

  // High-level function to initialize the SPI bus and set up the display.
  // Arguments:
  // orientation (int) = A value given 0-3 describing the landscape/portrait
  // rotation of the display. The orientations are display dependent.
  st7789_init_bus_and_display(1);

  // CB notifies lvgl when async dma spi transfer is complete to lvgl can start
  // using the buffer again.
  // Using dependency injection here so there's no st7789 compile dependency on
  // lvgl.
  st7789_spi_register_flush_cb(flush_callback);

  light.init(GPIO_NUM_2);

  // Run GUI on core 1.
  xTaskCreatePinnedToCore(guiLoop, "gui-loop", 4096 * 2, NULL, 3, NULL,
                          GUI_PROCESSOR_CORE_ID);

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
    lv_label_set_text_fmt(label, "%d", encoder->value);
    ESP_LOGI(TAG, "encoder value changed");
    light.set_duty(encoder->value);
    light.update();
  }
  float d = hcsr04.sample();
  filter.filter_sample(d);

  // Display distance for testing.
  // auto dist = static_cast<unsigned int>(d);
  // lv_label_set_text_fmt(label, "%d", dist);
  // ESP_LOGI(TAG, "d: %0.2f", d);
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
  lv_init();

  // Let lvgl know where to update the ticks from.
  // Here I'm using FreeRTOS.
  lv_tick_set_cb(get_milliseconds);

  disp = lv_display_create(CONFIG_DISPLAY_HEIGHT_PX, CONFIG_DISPLAY_WIDTH_PX);

  // Give lvgl our function to write to the display.
  lv_display_set_flush_cb(disp, flush);

  // Initialize the lvgl screen buffers.
  lv_color_t *buf1 =
      (lv_color_t *)heap_caps_aligned_alloc(4, BUF_SIZE, MALLOC_CAP_DMA);
  assert(buf1 != NULL);

  lv_color_t *buf2 =
      (lv_color_t *)heap_caps_aligned_alloc(4, BUF_SIZE, MALLOC_CAP_DMA);
  assert(buf2 != NULL);

  lv_display_set_buffers(disp, buf1, buf2, BUF_SIZE,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);

  // Draw the demo widgets to the screen.
  create_demo_widgets();

  // float d = ultrasonic.sample();
  // ESP_LOGI(TAG, "dist: %0.2f", d);

  while (1) {
    // Try to take the semaphore, call lvgl related function on success.
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {

      // I use my own timing here because its easier to control which core this
      // loop runs on with 'xTaskCreatePinnedToCore' and monitor how long my
      // loop takes for performance testing.
      const int64_t start = esp_timer_get_time();

      sample_inputs();

      handle_events();

      lv_task_handler();

      xSemaphoreGive(xGuiSemaphore);

      // Suspend the main loop until the next.
      const int64_t task_duration_us = (esp_timer_get_time() - start);
      ESP_LOGV(TAG, "gui loop delta us: %" PRIu64, task_duration_us);

      if (task_duration_us <= CONFIG_LV_DEF_REFR_PERIOD * 1000) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_LV_DEF_REFR_PERIOD -
                                 (task_duration_us / 1000)));
      } else {
        ESP_LOGW(TAG, "gui loop duration exceded refresh period: %" PRIu64 "us",
                 task_duration_us);
      }
    }

    // This is not necessary because gui thread already runs at the lowest
    // priority and can be interrupted. Its okay for my scheduler to spin in
    // this thread if my other interupts return to higher priority threads.
    // task_delay_ms(LV_DEF_REFR_PERIOD);
  }

  // A task should NEVER return.
  free(buf1);
  free(buf2);
  vTaskDelete(NULL);
}

static void flush_callback() { lv_display_flush_ready(disp); }
static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  UNUSED(disp);
  const st7789_area_t area_wrap = {
      .x1 = area->x1, .x2 = area->x2, .y1 = area->y1, .y2 = area->y2};
  st7789_flush(&area_wrap, (const uint8_t *)px_map);
}

/* Called from task. Note: don't cal lvgl api from interupt. Always use a helper
 * task to call lvgl. The interupt wakes up the helper task.*/
// static void counter_callback(re_count_t count) {
//   if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
//     lv_label_set_text_fmt(label, "%d", count);
//     xSemaphoreGive(xGuiSemaphore);
//   }
// }

static void button_callback(void *) { rep_reset(encoder); }

static void create_demo_widgets(void) {
  ESP_LOGD(TAG, "creating demo widgets");

  // Change the active screen's background color.
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000),
                            LV_PART_MAIN);

  // Create a white label, set its text and align it to the center.
  label = lv_label_create(lv_screen_active());
  lv_label_set_text_fmt(label, "0");
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff),
                              LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}