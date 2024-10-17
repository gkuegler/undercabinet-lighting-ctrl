#pragma once

#include <array>

#include "esp_log.h"
#include "event.hpp"

enum class State : int { UNDECIDED, HAND_OUT, HAND_IN };

/**
 * A sliding window filter to generate hand in/out event.
 * Ignores outliers.
 */
template <typename T, float THRESHOLD, size_t WINDOW_SIZE,
          size_t VALID_SAMPLES_NEEDED, size_t DEBOUNCE_COUNT>
class HandGestureFilter {
public:
  HandGestureFilter(){};
  ~HandGestureFilter(){};

  void init(EventQueue *event_q) { _event_q = event_q; };

  // Filter the sample and generate necessary events.
  void filter_sample(T sample) {
    // Push sample to circ buff and increment window.
    push(sample);

    // Deboundce the state change by skipping samples.
    if (_debounce_cnt > 0) {
      --_debounce_cnt;
      return;
    }

    State hand_pos = hand_position();

    if (State::HAND_OUT == _state && State::HAND_IN == hand_pos) {
      _state = State::HAND_IN;
      _event_q->post(Event::EVENT_HAND_ENTER);

    } else if (State::HAND_IN == _state && State::HAND_OUT == hand_pos) {
      _state = State::HAND_OUT;
      _event_q->post(Event::EVENT_HAND_EXIT);
      // Reset the debounce counter
      _debounce_cnt = DEBOUNCE_COUNT;
    }
  };

private:
  size_t _debounce_cnt = 0;

  size_t _index = 0;
  std::array<T, WINDOW_SIZE> _sample_window;

  T _threshold = THRESHOLD;
  EventQueue *_event_q = NULL;
  State _state = State::HAND_OUT;

  /**
   * Return true if the filtered samples fall below the threshold.
   */
  State hand_position() {
    size_t in = 0;
    size_t out = 0;
    for (const auto &s : _sample_window) {
      // Ignore invalid measurement.
      if (s == 0.0) {
        continue;
      }
      if (s < _threshold) {
        ++in;
      } else if (s > _threshold) {
        ++out;
      }
    }
    if (in > VALID_SAMPLES_NEEDED) {
      return State::HAND_IN;
    } else if (out > VALID_SAMPLES_NEEDED) {
      return State::HAND_OUT;
    } else {
      return State::UNDECIDED;
    }
  }

  void push(T item) {
    _sample_window[_index] = item;
    // Increment index wrapping back to the beginning.
    _index = (_index >= WINDOW_SIZE - 1) ? 0 : (_index + 1);
  }
};
