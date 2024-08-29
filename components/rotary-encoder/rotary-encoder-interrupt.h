#ifndef ROTARY_ENCODER_INTERUPT_H
#define ROTARY_ENCODER_INTERUPT_H

#include <stdbool.h>

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Underlying data type of the value.
 */
typedef int re_count_t;

/**
 * Function called when the value changes.
 */
typedef void (*re_callback_t)(re_count_t);

#define CONFIG_RE_DEFAULT_STEP_SIZE 1
#define CONFIG_RE_DEFAULT_OVERFLOW false
#define CONFIG_RE_DEFAULT_PULSES_PER_DETENT 4
#define CONFIG_RE_DEFAULT_MIN_PULSE_DURATION_NS 2000
#define CONFIG_RE_DEFAULT_CB_TASK_PRIORITY 5
#define CONFIG_RE_DEFAULT_CB_TASK_STACK_SIZE configMINIMAL_STACK_SIZE * 3

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  char name;
  int max;
  int min;

  // How much to increase or decrease the counter with each detent.
  int step_size;

  // If overflow is set to true, the value will wrap around after max or min.
  bool overflow;
  re_callback_t cb;
  int pinA;
  int pinB;

  // Sets the maximum glitch width, in nano seconds. If a signal pulse's width
  // is smaller than this value, then it will be treated as noise and will not
  // increase/decrease the internal counter.
  int min_pulse_duration_ns;
  int cb_task_priority;
  int cb_task_stack_size;

  _Atomic re_count_t value;          // private
  re_count_t stable_count;           // private
  re_count_t previous_count;         // private
  pcnt_unit_handle_t pcnt_unit;      // private
  TaskHandle_t counter_handler_task; // private
} re_interrupt_rotary_encoder_t;

/**
 * Note:
 * The max can't be greater than the MAX_VALUE(re_count_t) + step_size.
 * The min can't be less than the MIN_VALUE(re_count_t) - step_size.
 */
re_interrupt_rotary_encoder_t *
re_interrupt_rotary_encoder_initialize(re_interrupt_rotary_encoder_t *this);

void re_interrupt_encoder_set_value(re_interrupt_rotary_encoder_t *this,
                                    re_count_t value, bool notify_cb);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // ROTARY_ENCODER_INTERUPT_H