#pragma once

#include "esp_log.h"
#include "event.hpp"
#include "freertos/FreeRTOS.h"

/* Local Includes */
#include "ring-buffer.hpp"

enum class HandEvent
{
  NONE,
  ENTER,
  EXIT,

  // When an object has been placed within the target threshold for too long.
  OBSTRUCTED,
  OBSTRUCTION_CLEARED
};

/**
 * A sliding window filter to generate hand in/out events.
 */
template<typename T,
         T THRESHOLD_DIST_CM,
         size_t WINDOW_SIZE,
         size_t VALID_SAMPLES_NEEDED,
         int DEBOUNCE_PERIOD_MS>
class HandDetectionFilter
{
private:
  enum class HandState : int
  {
    UNDECIDED = 0,
    OUT,
    IN
  };

  RingBuffer<T, WINDOW_SIZE> _sample_window;
  TickType_t _ticks_last_change = 0;
  T _threshold_dist_cm = THRESHOLD_DIST_CM;
  HandState _state = HandState::OUT;
  int _obstruction_timeout_ms = 0;

public:
  HandDetectionFilter() {};
  ~HandDetectionFilter() {};

  /**
   * Filter the sample and return necessary events.
   */
  HandEvent process_sample(T sample, TickType_t ticks)
  {
    _sample_window.push(sample);

    if (_state == HandState::IN &&
        ticks - _ticks_last_change > pdMS_TO_TICKS(_obstruction_timeout_ms)) {
      return HandEvent::OBSTRUCTED;
    } else if (_state == HandState::OUT &&
               ticks - _ticks_last_change >
                 pdMS_TO_TICKS(_obstruction_timeout_ms)) {
      return HandEvent::OBSTRUCTION_CLEARED;
    }

    if (ticks - _ticks_last_change < pdMS_TO_TICKS(DEBOUNCE_PERIOD_MS)) {
      return HandEvent::NONE;
    }

    HandState hs = eval_hand_presence();

    // Evaluate state change.
    if (HandState::OUT == _state && HandState::IN == hs) {
      _state = HandState::IN;
      _ticks_last_change = ticks;
      return HandEvent::ENTER;
    } else if (HandState::IN == _state && HandState::OUT == hs) {
      _state = HandState::OUT;
      _ticks_last_change = ticks;
      return HandEvent::EXIT;
    } else {
      return HandEvent::NONE;
    }
  };

  // TODO: store in NVM; also make a factory reset?
  void set_threshold(float sp)
  {
    _threshold_dist_cm = sp;
    // Reset sample buffer to prevent false triggers on resume.
    _sample_window.buf.fill(0.0f);
  }

  /**
   * Return true if the filtered samples fall below the threshold.
   */
  HandState eval_hand_presence()
  {
    size_t cnt_below = 0;
    size_t cnt_above = 0;

    // Count samples above and below the threshold distance.
    for (size_t i = 0; i < _sample_window.size; i++) {
      auto s = _sample_window.buf[i];

      // Ignore an invalid measurement.
      if (s == 0.0) {
        continue;
      }

      if (s < _threshold_dist_cm) {
        ++cnt_below;
      } else if (s > _threshold_dist_cm) {
        ++cnt_above;
      }
    }

    if (cnt_below > VALID_SAMPLES_NEEDED) {
      return HandState::IN;
    } else if (cnt_above > VALID_SAMPLES_NEEDED) {
      return HandState::OUT;
    } else {
      return HandState::UNDECIDED;
    }
  }
};

template<size_t SIZE>
class ThresholdSet
{
public:
  int count = 0;
  RingBuffer<float, SIZE> buf;

  ThresholdSet() {};
  ~ThresholdSet() {};

  void sample(float d)
  {
    count++;
    buf.push(d);
  }
  float get_average()
  {
    if (count >= SIZE) {
      float avg;
      for (size_t i = 0; i < SIZE; i++) {
        avg += buf[i];
      }
      return avg / SIZE;
    }
  }
};
