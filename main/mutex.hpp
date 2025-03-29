#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class StaticMutex
{
public:
  StaticMutex() {};
  ~StaticMutex() {};

  void init()
  {
    handle = xSemaphoreCreateMutexStatic(&pxMutexBuffer);
    if (NULL == handle) {
      ESP_LOGE(TAG, "Could not create timer.");
      abort();
    } else {
      ESP_LOGI(TAG, "Timer created successfully.");
    }
  }

  bool take(TickType_t xTicksToWait = portMAX_DELAY)
  {
    return pdTRUE == xSemaphoreTake(handle, xTicksToWait = portMAX_DELAY);
  }

  bool give() { return pdTRUE == xSemaphoreGive(handle); }

private:
  static constexpr const char* TAG = "StaticMutex";
  SemaphoreHandle_t handle;
  StaticSemaphore_t pxMutexBuffer;
};
