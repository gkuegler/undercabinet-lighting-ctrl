#include "inttypes.h"
#include <numeric>

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*** LOCAL COMPONENT INCLUDES ***/
#include "debounce.hpp"
#include "ultrasonic.hpp"

/*** LOCAL INCLUDES ***/
#include "event.hpp"
#include "filter.hpp"
#include "led.hpp"
#include "mutex.hpp"
#include "parameters.h"
#include "ring-buffer.hpp"
#include "timer.hpp"

enum RANGING_MODE
{
  STOP,
  LED_CONTROL,
  SET_THRESHOLD_DISTANCE,
  STOP_SET_THRESHOLD_DISTANCE,
  FINISH_SET_THRESHOLD_DISTANCE
};

/*** STATIC PROTOTYPES ***/
static void
ultrasonic_ranging_task(void*);
static void
hmi_loop(void*);
static void
led_flash_task(void*);
void
start_led_flash();
void
stop_led_flash();
void set_thresh_dist_completed_cb(TimerHandle_t);

/*** GLOBAL VARIABLES ***/
static const char* TAG = "[main]";
TaskHandle_t ultrasonic_ranging_task_handle = NULL;
TaskHandle_t hmi_loop_handle = NULL;
TaskHandle_t flash_task_handle = NULL;

StaticQueue<Event, EVENT_QUEUE_COUNT> eventq;

DebouncedButton btn{ CONFIG_BUTTON_PIN1 };
static HCSR04 hcsr04;
static HandDetectionFilter<float,
                           HAND_DIST_THRESHOLD_CM,
                           HAND_SAMPLE_WINDOW_SIZE,
                           HAND_VALID_SAMPLES_NEEDED,
                           HAND_DEBOUNCE_COUNT>
  hand_detect_filter;
static RingBuffer<float, 20> thresh_set_buf;
static Led<CONFIG_LED_CTRL_PIN> ledctrl;

StaticTimer<SECONDS(10), set_thresh_dist_completed_cb> set_thresh_dist_timer;

std::atomic<uint32_t> ranging_mode = LED_CONTROL;
std::atomic<bool> g_enable_flash = false;

extern "C" void
app_main(void)
{
  eventq.init();
  ledctrl.init();
  set_thresh_dist_timer.init("CALT1");

  BaseType_t xResult;

  // For debugging purposes.
  esp_intr_dump(NULL);

  xResult = xTaskCreatePinnedToCore(ultrasonic_ranging_task,
                                    "ultrasonic-loop",
                                    4096 * 2,
                                    NULL,
                                    ULTRASONIC_TASK_PRIORITY,
                                    &ultrasonic_ranging_task_handle,
                                    ULTRASONIC_TASK_CORE_ID);

  if (xResult != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task: %d", xResult);
  }

  xResult = xTaskCreatePinnedToCore(hmi_loop,
                                    "hmi-loop",
                                    4096 * 2,
                                    NULL,
                                    HMI_LOOP_TASK_PRIORITY,
                                    &hmi_loop_handle,
                                    HMI_TASK_CORE_ID);
  if (xResult != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task: %d", xResult);
  }

  xResult = xTaskCreate(
    led_flash_task, "led-flash", 4096, NULL, 12, &flash_task_handle);
  if (xResult != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task: %d", xResult);
  }

  return;
}

static void
hmi_loop(void* pvParameter)
{
  const char* tag = "hmi";

  // Initialize button.
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << (int)CONFIG_BUTTON_PIN1);
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  /*
  dimming - single click
  grocery mode - long press; then single
  setting distance
  */
  RingBuffer<float, 8> led_bright_values;
  for (auto x : { 0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.0 }) {
    led_bright_values.push(x);
  }

  int bmode = 0;
  Event event;

  for (;;) {
    auto ticks = xTaskGetTickCount();

    // Not using LED timouts at the moment.
    // ledctrl.update_timeout_tick();

    // --- PROCESS BUTTON ---
    switch (btn.sample(ticks)) {
      case ButtonEvent::LongPress:
        // ESP_LOGD(tag, "changing button mode: %d", bmode);
        // change it so that the button wont accept multiple clicks unless in
        // the long press mode
        // bmode = !bmode;
        eventq.send(Event::START_SET_THRESHOLD_DISTANCE);
        break;

      case ButtonEvent::Click1:
        if (bmode == 0) {
          eventq.send(Event::CYCLE_BRIGHTNESS);
        } else if (bmode == 1) {
          eventq.send(Event::TOGGLE_LED);
        }
        break;

      case ButtonEvent::Click4:
        eventq.send(Event::CHIRP);
        break;

      default:
        break;
    }

    // Process global events.
    if (eventq.receive(event, 0)) {
      switch (event) {
        case Event::NONE:
          break;

        case Event::TOGGLE_LED:
          ledctrl.toggle();
          break;

        case Event::CYCLE_BRIGHTNESS: {
          auto bright = led_bright_values.next();
          // Notify user max brightness reached.
          if (bright == 1.0f) {
            ledctrl.pulse(1, 200, 20);
            vTaskDelay(pdMS_TO_TICKS(50));
            // ledctrl.pulse(1, 150, 50);
          }
          ledctrl.set_brightness_user(bright);
          break;
        }

        case Event::START_SET_THRESHOLD_DISTANCE:
          ESP_LOGD(TAG, "begin calibration");
          ranging_mode = SET_THRESHOLD_DISTANCE;
          start_led_flash();
          set_thresh_dist_timer.restart();
          break;

        case Event::CHIRP:
          ESP_LOGD(tag, "Chirp!");
          vTaskDelay(2000);
          ledctrl.pulse(1, 20, 150);
          ledctrl.pulse(1, 50, 150);
          ledctrl.resume();
          break;

        default:
          ESP_LOGE(TAG, "Unknown event: %d", (int)event);
          break;
      }
    }

    vTaskDelayUntil(&ticks, pdMS_TO_TICKS(1));
  }
}

void
start_led_flash()
{
  g_enable_flash = true;
  vTaskResume(flash_task_handle);
}

void
stop_led_flash()
{
  g_enable_flash = false;
}

static void
led_flash_task(void* pvParameter)
{
  for (;;) {
    if (g_enable_flash) {
      ledctrl.set_brightness(1.0f);
      vTaskDelay(1000);
      ledctrl.set_brightness(0.0f);
      vTaskDelay(500);
    } else {
      ledctrl.resume();
      vTaskSuspend(NULL);
    }
  }
}

static void
ultrasonic_ranging_task(void* pvParameter)
{
  (void)pvParameter; // not used

  const char* tag = "ultra";

  hcsr04.init(
    CONFIG_US_TRIG_PIN, CONFIG_US_ECHO_PIN, xTaskGetCurrentTaskHandle(), 0);

  while (true) {

    float dist = hcsr04.range_and_wait();

    auto ticks = xTaskGetTickCount();

    switch (ranging_mode) {
      case LED_CONTROL:
        // ????
        // Lockout if hand has been present for a while. 'Grocery Mode'
        // ???
        if (hand_detect_filter.process_sample(dist, ticks) ==
            Event::HAND_ENTER) {
          eventq.send(Event::TOGGLE_LED);
        }
        break;

      case SET_THRESHOLD_DISTANCE:
        // Add samples to ring buffer for the window average.
        thresh_set_buf.push(dist);

        // Slow down measurements so my buffer is meaningfull.
        vTaskDelay(pdMS_TO_TICKS(HAND_THRESH_SET_SAMPLE_INTERVAL_MS));
        break;

      // Process and accept the new setp.
      case FINISH_SET_THRESHOLD_DISTANCE: {
        ranging_mode = LED_CONTROL;
        auto& buf = thresh_set_buf.buf;
        float avg = std::accumulate(buf.begin(), buf.end(), 0.0) / buf.size();
        ESP_LOGD(tag, "New hand threshold setp: %.2fcm", avg);
        hand_detect_filter.set_threshold(avg);
        break;
      }
      default:
        break;
    }
#ifdef TESTING_SLOW_ULTRASONIC_POLLING_PERIOD
    ESP_LOGI(tag, "Range: %.2f", dist);
    vTaskDelayUntil(&ticks,
                    pdMS_TO_TICKS(TESTING_SLOW_ULTRASONIC_POLLING_PERIOD));
#endif // TESTING_SLOW_ULTRASONIC_POLLING_PERIOD
  }

  // Normally unreachable.
  vTaskDelete(NULL);
}

void
set_thresh_dist_completed_cb(TimerHandle_t xTimer)
{
  // TODO: fix ugly and poorly timed light flashing at end of threshold set
  ranging_mode = FINISH_SET_THRESHOLD_DISTANCE;
  stop_led_flash();
  eventq.send(Event::CHIRP);
}
