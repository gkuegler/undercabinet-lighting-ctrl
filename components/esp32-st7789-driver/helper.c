#include "helper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MS_TO_TICKS(x) (x / portTICK_PERIOD_MS)
TickType_t ms_to_ticks(int ms) { return ms / portTICK_PERIOD_MS; }
void task_delay_ms(int ms) { vTaskDelay(ms / portTICK_PERIOD_MS); }