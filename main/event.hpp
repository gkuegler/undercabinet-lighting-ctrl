#pragma once

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum class Event : int { EVENT_HAND_EXIT, EVENT_HAND_ENTER };
enum class State : int { UNDECIDED, HAND_OUT, HAND_IN };

/*
I can't have templates here if I want to use the full class another object.*/
// template <size_t SIZE>
class EventQueue {
public:
  EventQueue(size_t SIZE_, uint8_t *ucQueueStorage_) {
    SIZE = SIZE_;
    ucQueueStorage = ucQueueStorage_;
  }
  ~EventQueue(){};

  void init() {
    event_q =
        xQueueCreateStatic(SIZE, sizeof(Event), ucQueueStorage, &xQueueBuffer);
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
  static constexpr const char *TAG = "Queue";
  size_t SIZE;

  /* The variable used to hold the queue's data structure. */
  StaticQueue_t xQueueBuffer;
  
  /* The array to use as the queue's storage area. This must be at least
   * uxQueueLength * uxItemSize bytes. */
  uint8_t *ucQueueStorage;

  QueueHandle_t event_q = nullptr;
};