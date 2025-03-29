#pragma once

#define LOGPOINT ESP_LOGE("BRK", "%s: %d", __FILE__, __LINE__);

#define MINUTES(x) (x * 60 * 1000)
#define SECONDS(x) (x * 1000)

#define HMI_POLLING_PERIOD_MS 1
#define EVENT_QUEUE_COUNT     5

#define CONFIG_US_STARTING_DELAY_BETWEEN_RANGES_MS 3

#define HAND_DIST_THRESHOLD_CM    15.0f
#define HAND_SAMPLE_WINDOW_SIZE   5
#define HAND_VALID_SAMPLES_NEEDED 4
#define HAND_DEBOUNCE_COUNT       (int)(pdMS_TO_TICKS(300))

// Total time the unit will spend in the threshold set mode.
#define HAND_THRESH_SET_TIME_MS            10 * 1000
#define HAND_THRESH_SET_SAMPLE_INTERVAL_MS 50

// How long the hand must remain still for the threshold distance to be
// accepted.
#define HAND_THRESH_SET_AVG_WINDOW_MS 3 * 1000

#define LED_PWM_FREQ_HZ        (40 * 1000)
#define LED_WARNING_TIMEOUT_MS MINUTES(120)
#define LED_SHUTOFF_TIMEOUT_MS (LED_WARNING_TIMEOUT_MS + MINUTES(10))

#define MODULE_QTPY_ESP32_S3_PROTOTYPE_1
#ifdef MODULE_QTPY_ESP32_S3_PROTOTYPE_1
#define CONFIG_LED_CTRL_PIN   GPIO_NUM_18
#define CONFIG_ROTARY_PIN_BTN GPIO_NUM_17
#define CONFIG_ROTARY_PIN_A   GPIO_NUM_8
#define CONFIG_ROTARY_PIN_B   GPIO_NUM_9
#define CONFIG_US_TRIG_PIN    GPIO_NUM_16
#define CONFIG_US_ECHO_PIN    GPIO_NUM_6
#define CONFIG_BUTTON_PIN1    GPIO_NUM_0
#endif // MODULE_QTPY_ESP32_S3_PROTOTYPE_1
#ifdef MODULE_QTPY_ESP32_S3_TEST_UNIT
#define CONFIG_LED_RELAY_PIN GPIO_NUM_18
#define CONFIG_TRIGGER_PIN   GPIO_NUM_16
#define CONFIG_ECHO_PIN      GPIO_NUM_6
#define CONFIG_BUTTON_PIN1   GPIO_NUM_0
#endif // MODULE_QTPY_ESP32_S3_PROTOTYPE_1

// TASK PRIORITIES
// ESP32 has 25 task priorities. (0-24). Higher number means higher priority.
#define ESP32_MAX_PRIORITY configMAX_PRIORITIES - 1
static_assert(ESP32_MAX_PRIORITY <= 24);
#define ULTRASONIC_TASK_PRIORITY 24
#define HMI_LOOP_TASK_PRIORITY   5
#define ULTRASONIC_TASK_CORE_ID  0
#define HMI_TASK_CORE_ID         1

/*** TESTING DEFINES ***/
// #define TEST_LOG_DISTANCES // log distance measurements to console
// #define TESTING_SLOW_ULTRASONIC_POLLING_PERIOD 750

// TODO: check task stats with vTaskGetRunTimeStats(char *pcWriteBuffer)
// Use a buffer of >40 characters