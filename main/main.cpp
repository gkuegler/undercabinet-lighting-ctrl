#include "inttypes.h"
#include <numeric>

#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*** LOCAL COMPONENT INCLUDES ***/
#include "debounce.hpp"
#include "ultrasonic.hpp"

/*** LOCAL INCLUDES ***/
#include "dipswitch.h"
#include "error.h"
#include "event.hpp"
#include "filter.hpp"
#include "led.hpp"
#include "mutex.hpp"
#include "parameters.h"
#include "ring-buffer.hpp"
#include "timer.hpp"

// TODO: make options with jumpers and/or wifi server
// TODO: do enclosure
// TODO: enable OTA updates?

enum class UltrasonicMode : int
{
  LedControl,
  SetThreshold,
  FinishSetThreshold
};

/*** STATIC PROTOTYPES ***/
// Tasks
static void
us_ranging_task(void*);
static void
hmi_task(void*);
static void
led_flash_task(void*);

static void set_thresh_dist_completed_cb(TimerHandle_t);

/*** GLOBAL VARIABLES ***/
static const char* TAG = "main.cpp";

static TaskHandle_t ghRangeTask = NULL;
static TaskHandle_t ghHmiTask = NULL;
static TaskHandle_t ghLedFlashTask = NULL;

static StaticQueue<Event, EVENT_QUEUE_COUNT> eventq;
static DebouncedButton btn{ CONFIG_BTN_PIN1, GpioPullDirection::Up };
static HCSR04 hcsr04; // Ultrasonic ranging peripheral.
static HandDetectionFilter<float,
                           HAND_DEFAULT_DIST_THRESH_CM,
                           HAND_SAMPLE_WINDOW_SIZE,
                           HAND_VALID_SAMPLES_NEEDED,
                           HAND_DEBOUNCE_PERIOD_MS>
  range_filter;
static RingBuffer<float, HAND_THRESH_SET_BUFSIZE> thresh_set_buf;
static Led led{ CONFIG_LED_CTRL_PIN };
static StaticTimer<HAND_THRESH_SET_DURATION_MS, set_thresh_dist_completed_cb>
  set_thresh_dist_timer{ "timer1" };
static DipSwitches dip;

static std::atomic<UltrasonicMode> gRangingMode = UltrasonicMode::LedControl;
static std::atomic<bool> gSensorObstructed = false;
static std::atomic<bool> gEnableFlash = false;

extern "C" void
app_main(void)
{
#ifdef TEST_FAKE_DIP_SWITCHES
  dip.enable_led_pwm = true;
  // dip.enable_led_shutoff_timeout = false;
  // dip.enable_grocery_detection = false;
  // dip.failsafe_manual_switch_mode = false;
#else
  read_dip_values(&dip);
#endif // TEST_FAKE_DIP_SWITCHES

  eventq.init();
  led.init(dip.enable_led_pwm);
  led.safety_timeout_enabled = dip.enable_led_shutoff_timeout;

  set_thresh_dist_timer.init();

  btn.init();
  btn.max_multi_clicks = 1; // for bmode 0

  freertos_error_check_abort(xTaskCreatePinnedToCore(us_ranging_task,
                                                     "ultrasonic-loop",
                                                     4096 * 2,
                                                     NULL,
                                                     ULTRASONIC_TASK_PRIORITY,
                                                     &ghRangeTask,
                                                     ULTRASONIC_TASK_CORE_ID),
                             TAG,
                             "create us_ranging_task");

  freertos_error_check_abort(xTaskCreatePinnedToCore(hmi_task,
                                                     "hmi-loop",
                                                     4096 * 2,
                                                     NULL,
                                                     HMI_LOOP_TASK_PRIORITY,
                                                     &ghHmiTask,
                                                     HMI_TASK_CORE_ID),
                             TAG,
                             "create hmi_task");

  freertos_error_check_abort(
    xTaskCreate(
      led_flash_task, "led_flash_task", 4096, NULL, 12, &ghLedFlashTask),
    TAG,
    "create led_flash_task");

  return;

  // For debugging purposes.
  esp_intr_dump(NULL);
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
      bmode = !bmode;
      btn.max_multi_clicks = bmode ? 4 : 1;
      eventq.send(Event::CHIRP_DOUBLE);
      break;

    case 1:
      // Re-enable normal led control of ultrsonic sensor.
      gSensorObstructed = false;

      if (bmode == 0) {
        eventq.send(Event::CYCLE_BRIGHTNESS);
      } else if (bmode == 1) {
        eventq.send(Event::TOGGLE_LED);
      }
      break;

    case 4:
      eventq.send(Event::START_SET_THRESHOLD_DISTANCE);
      break;

    default:
      break;
  }
}

static void
hmi_task(void* pvParameter)
{
  const char* tag = "HMI";

  RingBuffer<float, 8> led_bright_values;
  for (auto x : { 0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.0 }) {
    led_bright_values.push(x);
  }

  Event event;

  while (true) {
    auto ticks = xTaskGetTickCount();

    led.check_safety_timeout(ticks);

    sample_button(ticks);

    // Not using LED timouts at the moment.
    // led.update_timeout_tick();

    // Process global events.
    if (eventq.receive(event, 0)) {
      switch (event) {

        case Event::TOGGLE_LED:
          led.toggle();
          break;

        case Event::RESUME_LED:
          led.resume();
          break;

        case Event::CYCLE_BRIGHTNESS: {
          if (led.pwm_enabled) {
            auto bright = led_bright_values.next();
            // Notify user max brightness reached.
            if (bright == 1.0f) {
              eventq.send(Event::CHIRP_SINGLE);
            }
            led.set_brightness_user(bright);
          } else {
            led.toggle();
          }
          break;
        }

        case Event::START_SET_THRESHOLD_DISTANCE:
          ESP_LOGD(TAG, "begin calibration");
          gRangingMode = UltrasonicMode::SetThreshold;
          vTaskResume(ghLedFlashTask);
          set_thresh_dist_timer.restart();
          break;

        case Event::CHIRP_SINGLE:
          led.pulse(1, 150, 50);
          vTaskDelay(pdMS_TO_TICKS(50));
          led.resume();
          break;
        case Event::CHIRP_DOUBLE:
          if (led.pwm_enabled) {
            led.pulse(2, 150, 50);
            vTaskDelay(pdMS_TO_TICKS(50));
          } else {
            led.toggle();
            vTaskDelay(pdMS_TO_TICKS(1000));
            led.toggle();
          }
          led.resume();
          break;
      }
    }

    vTaskDelayUntil(&ticks, pdMS_TO_TICKS(1));
  }
}

static void
led_flash_task(void* pvParameter)
{
  vTaskSuspend(NULL);

  while (true) {
    led.set_brightness(1.0f);
    vTaskDelay(1000);
    led.set_brightness(0.0f);
    vTaskDelay(500);
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
      case UltrasonicMode::LedControl:
        switch (range_filter.process_sample(dist, ticks)) {
          case HandEvent::ENTER:
            if (!gSensorObstructed) {
              eventq.send(Event::TOGGLE_LED);
            }
            break;
          case HandEvent::OBSTRUCTED:
            // A button press shall reset this.
            if (dip.enable_grocery_detection) {
              gSensorObstructed = true;
            }
            break;
          default:
            break;
        }
        break;

      case UltrasonicMode::SetThreshold:
        // Add samples to ring buffer for the window average.
        thresh_set_buf.push(dist);

        // Slow down measurements so my buffer is meaningfull.
        vTaskDelay(pdMS_TO_TICKS(HAND_THRESH_SET_SAMPLE_INTERVAL_MS));
        break;

      // Process and accept the new setp.
      case UltrasonicMode::FinishSetThreshold: {
        gRangingMode = UltrasonicMode::LedControl;

        auto& buf = thresh_set_buf.buf;
        float avg = std::accumulate(buf.begin(), buf.end(), 0.0) / buf.size();
        range_filter.set_threshold(avg);
        ESP_LOGI(tag, "New hand threshold setp: %.2fcm", avg);

        vTaskSuspend(ghLedFlashTask);
        eventq.send(Event::RESUME_LED);
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
  gRangingMode = UltrasonicMode::FinishSetThreshold;
}
