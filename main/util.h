#pragma once

#define LOGPOINT ESP_LOGE("BRK", "%s: %d", __FILE__, __LINE__);

#define SECONDS_TO_MS(x) (x * 1000)
#define MINUTES_TO_MS(x) (x * 1000 * 60)
#define HOURS_TO_MS(x)   (x * 1000 * 60 * 60)