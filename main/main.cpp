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
#include "error.h"
#include "event.hpp"
#include "filter.hpp"
#include "led.hpp"
#include "mutex.hpp"
#include "parameters.h"
#include "ring-buffer.hpp"
#include "timer.hpp"

// TODO: rename and clean code for v1.3

// TODO: make options with jumpers and/or wifi server
// TODO: do enclosure
// TODO: enable OTA updates?

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
us_ranging_task(void*);
static void
hmi_task(void*);
static void
led_flash_task(void*);

void
start_led_flash();
void
stop_led_flash();
void set_thresh_dist_completed_cb(TimerHandle_t);

/*** GLOBAL VARIABLES ***/
static const char* TAG = "[main]";
TaskHandle_t hUsRanging = NULL;
TaskHandle_t hHmiTask = NULL;
TaskHandle_t hLedFlash = NULL;

StaticQueue<Event, EVENT_QUEUE_COUNT> qEvent;

// Todo don't make these templates.
static DebouncedButton btn{ CONFIG_BTN_PIN1, GpioPullDirection::Up };
static HCSR04 hcsr04;
static HandDetectionFilter<float,
                           HAND_DIST_THRESHOLD_CM,
                           HAND_SAMPLE_WINDOW_SIZE,
                           HAND_VALID_SAMPLES_NEEDED,
                           HAND_DEBOUNCE_COUNT>
  hand_detect_filter;

#define HAND_THRESH_SET_BUFSIZE 20

static RingBuffer<float, 20> thresh_set_buf;
static Led<CONFIG_LED_CTRL_PIN> ledctrl;

#define HAND_THRESH_SET_DURATION_MS SECONDS(5)

StaticTimer<HAND_THRESH_SET_DURATION_MS, set_thresh_dist_completed_cb>
  set_thresh_dist_timer;

std::atomic<uint32_t> gRangingMode = LED_CONTROL;
std::atomic<bool> gEnableFlash = false;

static_assert((HAND_THRESH_SET_SAMPLE_INTERVAL_MS * HAND_THRESH_SET_BUFSIZE *
               4) < HAND_THRESH_SET_DURATION_MS,
              "Buffer may be too large for accurate sample averaging.");

extern "C" void
app_main(void)
{
  qEvent.init();
  ledctrl.init();
  set_thresh_dist_timer.init("CALT1");

  btn.init();
  btn.max_multi_clicks = 1; // for bmode 0

  // For debugging purposes.
  // esp_intr_dump(NULL);

  freertos_error_check_abort(xTaskCreatePinnedToCore(us_ranging_task,
                                                     "ultrasonic-loop",
                                                     4096 * 2,
                                                     NULL,
                                                     ULTRASONIC_TASK_PRIORITY,
                                                     &hUsRanging,
                                                     ULTRASONIC_TASK_CORE_ID),
                             TAG,
                             "create us_ranging_task");

  freertos_error_check_abort(xTaskCreatePinnedToCore(hmi_task,
                                                     "hmi-loop",
                                                     4096 * 2,
                                                     NULL,
                                                     HMI_LOOP_TASK_PRIORITY,
                                                     &hHmiTask,
                                                     HMI_TASK_CORE_ID),
                             TAG,
                             "create hmi_task");

  freertos_error_check_abort(
    xTaskCreate(led_flash_task, "led_flash_task", 4096, NULL, 12, &hLedFlash),
    TAG,
    "create led_flash_task");

  return;
}

void
sample_button(TickType_t ticks)
{
  static const char* tag = "BTN";
  static int bmode = 0;

  // --- PROCESS BUTTON ---
  switch (btn.sample(ticks)) {
    case BUTTON_EVT_LONG_PRESS:
      ESP_LOGD(tag, "changing button mode: %d", bmode);
      // change it so that the button wont accept multiple clicks unless in
      // the long press mode
      bmode = !bmode;
      btn.max_multi_clicks = bmode ? 4 : 1;
      // Instead of chirp I could do a toggle?
      qEvent.send(Event::CHIRP_DOUBLE);
      break;

    case 1:
      if (bmode == 0) {
        qEvent.send(Event::CYCLE_BRIGHTNESS);
      } else if (bmode == 1) {
        qEvent.send(Event::TOGGLE_LED);
      }
      break;

    case 4:
      qEvent.send(Event::START_SET_THRESHOLD_DISTANCE);
      break;

    default:
      break;
  }
}

static void
hmi_task(void* pvParameter)
{
  const char* tag = "hmi";

  RingBuffer<float, 8> led_bright_values;
  for (auto x : { 0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.0 }) {
    led_bright_values.push(x);
  }

  Event event;

  for (;;) {
    auto ticks = xTaskGetTickCount();

    sample_button(ticks);

    // Not using LED timouts at the moment.
    // ledctrl.update_timeout_tick();

    // Process global events.
    if (qEvent.receive(event, 0)) {
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
            // TODO: fix chirp at top end
            ledctrl.pulse(1, 200, 20);
            vTaskDelay(pdMS_TO_TICKS(50));
            // ledctrl.pulse(1, 150, 50);
          }
          ledctrl.set_brightness_user(bright);
          break;
        }

        case Event::START_SET_THRESHOLD_DISTANCE:
          ESP_LOGD(TAG, "begin calibration");
          gRangingMode = SET_THRESHOLD_DISTANCE;
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
  gEnableFlash = true;
  vTaskResume(hLedFlash);
}

void
stop_led_flash()
{
  gEnableFlash = false;
}

static void
led_flash_task(void* pvParameter)
{
  while (true) {
    if (gEnableFlash) {
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
us_ranging_task(void* pvParameter)
{
  (void)pvParameter; // not used

  const char* tag = "ultra";

  hcsr04.init(
    CONFIG_US_TRIG_PIN, CONFIG_US_ECHO_PIN, xTaskGetCurrentTaskHandle(), 0);

  while (true) {

    float dist = hcsr04.range_and_wait();

    auto ticks = xTaskGetTickCount();

    switch (gRangingMode) {
      case LED_CONTROL:
        // TODO: grovery mode???
        // Lockout if hand has been present for a while. 'Grocery Mode'
        if (hand_detect_filter.process_sample(dist, ticks) ==
            Event::HAND_ENTER) {
          qEvent.send(Event::TOGGLE_LED);
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
        gRangingMode = LED_CONTROL;
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
  gRangingMode = FINISH_SET_THRESHOLD_DISTANCE;
  stop_led_flash();
  qEvent.send(Event::CHIRP);
}
