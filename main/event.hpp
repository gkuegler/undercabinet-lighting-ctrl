#pragma once

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum class Event : int { EVENT_HAND_EXIT, EVENT_HAND_ENTER };
enum class State : int { UNDECIDED, HAND_OUT, HAND_IN };

template <size_t SIZE> class EventQueue {
public:
  EventQueue();
  ~EventQueue();

  init() {
    event_q = xQueueCreateStatic(SIZE, sizeof(Event), ucQueueStorageArea,
                                 xStaticQueue);
    if (event_q == NULL) {
      ESP_LOGE(TAG, "Could not create event queue.");
      abort();
    }
  }

  void post(Event e) {
    Event item = e;
    BaseType_t result = xQueueSend(event_q, &item, 0);
    if (result == errQUEUE_FULL) {
      ESP_LOGE(TAG, "Event queue was full.");
    }
  }

  bool receive(Event &event) {
    if (pdTRUE == xQueueReceive(event_q, &event,
                                0 // Return immediately if empty.
                                )) {
      return true;
    } else {
      return false;
    }
  }

private:
  const char *TAG = "Queue";
  QueueHandle_t event_q = nullptr;

  /* The variable used to hold the queue's data structure. */
  StaticQueue_t xStaticQueue;
  /* The array to use as the queue's storage area. This must be at least
   * uxQueueLength * uxItemSize bytes. */
  uint8_t ucQueueStorageArea[SIZE * sizeof(Event)];
};