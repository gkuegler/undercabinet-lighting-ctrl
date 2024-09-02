#pragma once

#include <array>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// TODO: put this with the Q
enum class Event : int { EVENT_HAND_EXIT, EVENT_HAND_ENTER };
enum class State : int { HAND_OUT, HAND_IN, UNDECIDED };

QueueHandle_t create_event_q() {
  //   The queue length should hold at least as many items as the multiple of
  //   the sampling loop versus the GUI loop.
  QueueHandle_t event_q = xQueueCreate(10, sizeof(int));
  if (event_q == NULL) {
    ESP_LOGE("Queue", "Could not event create queue.");
    abort();
  }
  return event_q;
}

void q_event(QueueHandle_t q, Event e) {
  Event item = e;
  BaseType_t result = xQueueSend(q, &item, 0);
  if (result == errQUEUE_FULL) {
    ESP_LOGE("Queue", "Event queue was full.");
  }
}

/**
 * A sliding window filter ignores outliers will filter against errant data.
 */
template <typename T, float THRESHOLD, size_t N, size_t VALID, size_t SKIP>
class Filter {
public:
  Filter(){};
  ~Filter(){};

  void init(QueueHandle_t event_q) { _event_q = event_q; };

  // Filter the sample and generate necessary events.
  void filter_sample(T sample) {
    push(sample);

    // Deboundce the state change by skipping samples.
    if (_sample_skip_count > 0) {
      --_sample_skip_count;
      return;
    }

    State hand_pos = hand_position();

    if (State::HAND_OUT == _state && State::HAND_IN == hand_pos) {
      _state = State::HAND_IN;
      q_event(_event_q, Event::EVENT_HAND_ENTER);

    } else if (State::HAND_IN == _state && State::HAND_OUT == hand_pos) {
      _state = State::HAND_OUT;
      q_event(_event_q, Event::EVENT_HAND_EXIT);
      // Prevent sending the hand enter event too soon.
      _sample_skip_count = SKIP;
    }
  };

private:
  const size_t _size = N;
  const size_t _valid_sample_count = VALID;
  size_t _sample_skip_count = 0;

  size_t _index = 0;
  std::array<T, N> _sample_window;

  T _threshold = THRESHOLD;
  QueueHandle_t _event_q = NULL;
  State _state = State::HAND_OUT;

  /**
   * Return true if the filtered samples fall below the threshold.
   */
  State hand_position() {
    size_t in = 0;
    size_t out = 0;
    for (const auto &s : _sample_window) {
      if (s == 0.0) {
        continue;
      }
      if (s < _threshold) {
        ++in;
      } else if (s > _threshold) {
        ++out;
      }
    }
    if (in > _valid_sample_count) {
      return State::HAND_IN;
    } else if (out > _valid_sample_count) {
      return State::HAND_OUT;
    } else {
      return State::UNDECIDED;
    }
  }

  void push(T item) {
    _sample_window[_index] = item;
    // Increment index wrapping back to the beginning.
    _index = (_index >= N - 1) ? 0 : (_index + 1);
  }
};
