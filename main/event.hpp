#pragma once

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum class Event : int
{
  // HAND_PRESENCE_TIMEOUT,
  START_SET_THRESHOLD_DISTANCE,
  // COMPLETE_SET_THRESHOLD_DISTANCE,
  TOGGLE_LED,
  RESUME_LED,
  CYCLE_BRIGHTNESS,
  CHIRP_SINGLE,
  CHIRP_DOUBLE
};

/* Class wrapper around a FreeRTOS StaticQueue.*/
template<typename TYPE, size_t COUNT>
class StaticQueue
{
private:
  static constexpr const char* TAG = "StaticQueue";
  /* The variable used to hold the queue's data structure. */
  StaticQueue_t xQueueBuffer;

  /* The array to use as the queue's storage area. This must be at least
   * uxQueueLength * uxItemSize bytes. */
  //  uint8_t* ucQueueStorage;
  uint8_t ucQueueStorage[COUNT * sizeof(TYPE)];

  QueueHandle_t hQueue = nullptr;

public:
  StaticQueue() {};
  ~StaticQueue() {};

  void init()
  {
    hQueue =
      xQueueCreateStatic(COUNT, sizeof(TYPE), ucQueueStorage, &xQueueBuffer);
    if (hQueue == NULL) {
      ESP_LOGE(TAG, "Could not create event queue.");
      abort();
    }
  }

  void send(TYPE item)
  {
    // TYPE _item = item;
    if (errQUEUE_FULL == xQueueSend(hQueue, &item, 0)) {
      ESP_LOGE(TAG, "Event queue was full.");
    }
  }

  bool receive(Event& event, TickType_t timeout = portMAX_DELAY)
  {
    return (pdTRUE == xQueueReceive(hQueue, &event, timeout));
  }
};