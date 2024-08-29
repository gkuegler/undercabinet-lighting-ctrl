#ifndef ROTARY_ENCODER_POLLING_H
#define ROTARY_ENCODER_POLLING_H

#include <stdatomic.h>
#include <stdbool.h>

#include "driver/pulse_cnt.h"

/**
 * Underlying data type of the count.
 */
typedef int re_count_t;

#define CONFIG_RE_DEFAULT_STEP_SIZE 1
#define CONFIG_RE_DEFAULT_OVERFLOW false
#define CONFIG_RE_DEFAULT_PULSES_PER_DETENT 4
#define CONFIG_RE_DEFAULT_MIN_PULSE_DURATION_NS 2000

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Note:
 * The max can't be greater than the MAX_VALUE(re_count_t) + step_size.
 * The min can't be less than the MIN_VALUE(re_count_t) - step_size.
 */
typedef struct {
  char name;
  int max;
  int min;

  // How much to increase or decrease the counter with each detent.
  int step_size;

  // If overflow is set to true, the count will wrap around after max or min.
  bool overflow;
  int pinA;
  int pinB;

  // Sets the maximum glitch width, in nano seconds. If a signal pulse's width
  // is smaller than this value, then it will be treated as noise and will not
  // increase/decrease the internal counter.
  int min_pulse_duration_ns;

  re_count_t value; // get only
  re_count_t delta; // get only
  bool changed;
  int16_t previous_pulse_count; // private
  pcnt_unit_handle_t pcnt_unit; // private
} re_polling_rotary_encoder_t;

bool rep_initialize(re_polling_rotary_encoder_t *self);
void rep_reset(re_polling_rotary_encoder_t *self);
re_count_t rep_get_value(re_polling_rotary_encoder_t *self);
re_count_t rep_get_delta(re_polling_rotary_encoder_t *self);
bool rep_sample(re_polling_rotary_encoder_t *self);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // ROTARY_ENCODER_POLLING_H