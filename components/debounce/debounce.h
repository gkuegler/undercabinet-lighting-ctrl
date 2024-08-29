#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <stdbool.h>
// #include <stdint.h>

// Response time is equal to sample period multiplied by the sample count.
#define CONFIG_DB_DEFAULT_SAMPLE_PERIOD_MS 5 // delay period  of the timer
#define CONFIG_DB_DEFAULT_SAMPLE_COUNT 12
#define CONFIG_DB_DEFAULT_SAMPLING_TASK_PRIORITY 6

typedef void (*edge_cb_t)(void *);

typedef enum { DB_EDGE_FALLING, DB_EDGE_RISING } edge_type_t;
typedef enum { DB_PIN_PULL_NONE, DB_PIN_PULLUP, DB_PIN_PULLDOWN } pin_pull_t;

/**
 * @param pin The GPIO pin number.
 * @param etype Specify rising or falling edge.
 * @param cb The callback function.
 * @note This will be executed in the context of a FreeRTOS task.
 * @param sample_period_ms The the time between samples in milliseconds.
 * @param sample_count The number of stable samples required to indicate a
 * change of state.
 * @note The response time is equal to sample period multiplied by the sample
 * count.
 * @param sampling_task_priority The FreeRTOS task priority asigned to the
 * sampling timer loop.
 */
typedef struct {
  edge_type_t etype;
  pin_pull_t pull_up;
  int pin;
  edge_cb_t cb;
  void *cb_params;
  int sample_period_ms;
  int sample_count;
  int sampling_task_priority;
  int cb_task_stack_size;
  int core_id;
} db_edge_input_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register a debounced edge callback on a GPIO pin. The callback will
 * be called after the output has remained stable for 'n' samples.
 * @param info The configuration structure for the pin.
 * @note The response time is equal to sample period multiplied by the sample
 * count.
 */
bool db_register_edge(db_edge_input_t *input);
// void db_set_callback(int pin, edge_cb_t cb);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // DEBOUNCE_H