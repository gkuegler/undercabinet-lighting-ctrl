
#include "error.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

template<size_t TIME_MS, TimerCallbackFunction_t CALLBACK>
class StaticTimer
{
private:
  static constexpr const char* tag = "StaticTimer";

  StaticTimer_t buf;
  TimerHandle_t handle;
  const char* name;

public:
  StaticTimer(const char* name)
    : name(name) {};
  ~StaticTimer() {};

  void init()
  {
    handle = xTimerCreateStatic(
      name, pdMS_TO_TICKS(TIME_MS), pdFALSE, 0, CALLBACK, &buf);
    if (NULL == handle) {
      ESP_LOGE(tag, "Could not create timer '%s'.", name);
      abort();
    } else {
      ESP_LOGD(tag, "Timer '%s' created successfully.", name);
    }
  }

  /* Starts or restarts the timer. */
  bool restart(TickType_t xBlockTime = portMAX_DELAY)
  {
    return pdTRUE == xTimerReset(handle, xBlockTime);
  }

  bool stop(TickType_t xBlockTime = portMAX_DELAY)
  {
    return pdTRUE == xTimerStop(handle, xBlockTime);
  }
};
