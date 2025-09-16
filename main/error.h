#pragma once

#include "esp_log.h"
#include "freertos/projdefs.h"

#include "esp_check.h"

// Not thread safe.
void
esp_error_check_abort(esp_err_t err, const char* tag, const char* title)
{
  static const size_t bufsize = 128;
  static char buf[bufsize];

  if (ESP_OK != err) {
    ESP_LOGE(tag,
             "Failure on '%s' with error: %s",
             title,
             esp_err_to_name_r(err, (char*)buf, bufsize));
    std::abort();
  }
}

void
freertos_error_check_abort(BaseType_t err, const char* tag, const char* title)
{
  switch (err) {
    case pdPASS:
      return;
      break;
    case errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY:
      ESP_LOGE(tag, "Could not allocate required memory for '%s'.", title);
      break;
    default:
      ESP_LOGE(tag, "Freertos call failed for '%s'.", title);
      break;
  }

  std::abort();
}