
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

template<size_t TIME_MS, TimerCallbackFunction_t CALLBACK>
class StaticTimer
{
public:
  StaticTimer() {};
  ~StaticTimer() {};

  void init(const char* tag)
  {
    handle = xTimerCreateStatic(
      tag, pdMS_TO_TICKS(TIME_MS), pdFALSE, 0, CALLBACK, &buf);
    if (NULL == handle) {
      ESP_LOGE(tag, "Could not create timer.");
      abort();
    } else {
      ESP_LOGI(tag, "Timer created successfully.");
    }
  }

  bool restart(TickType_t xBlockTime = portMAX_DELAY)
  {
    return pdTRUE == xTimerReset(handle, xBlockTime);
  }
  bool stop(TickType_t xBlockTime = portMAX_DELAY)
  {
    return pdTRUE == xTimerStop(handle, xBlockTime);
  }

private:
  StaticTimer_t buf;
  TimerHandle_t handle;
};
