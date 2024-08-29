/*
Use at least C++20.
ESP-IDF uses GNU++23 by default.

*/

#include "ultrasonic_rmt.hpp"

#include <assert.h>
// #include <functional>
// #include <stdatomic.h>
#include <atomic>
#include <cstring>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define BIT64(nr) (1ULL << (nr))

static const char *_TAG = "RMT";

// https://stackoverflow.com/questions/19532826/what-does-a-dangerous-relocation-error-mean

// IRAM attribute causing linking error: dangerous relocation: l32r: literal
// placed after use static bool IRAM_ATTR rmt_rx_done_callback(
bool IRAM_ATTR PulseMeasure::rmt_rx_done_callback(
    rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data,
    void *user_ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  auto self = static_cast<PulseMeasure *>(user_ctx);

  if (event_data->num_symbols > 0) {
    // Extract the duration of the high pulse
    uint16_t high_pulse_duration_us = event_data->received_symbols[0].duration0;
    // Conversion from time duration to target distance.
    // 0.01715 cm/us = (343 m/s) * (100 cm/m) * (1/1000000 s/us) * 0.5
    // (accounts for down and back)
    // Can't do floating-point math in an interrupt handler!
    // self->distance = high_pulse_duration_us * 0.01715f;

    self->_pulse_duration_us = high_pulse_duration_us;
    self->_pulse_in_flight = false;
    pulse_info_t p = {self->pinfo.tx_queued, high_pulse_duration_us,
                      esp_timer_get_time()};

    xQueueSendFromISR(self->_rx_q, &p, &xHigherPriorityTaskWoken);
  }

  return 0;
}

void PulseMeasure::init_rx_channel(gpio_num_t rx) {
  rmt_rx_channel_config_t rxcfg;
  memset(&rxcfg, 0, sizeof(rxcfg));
  rxcfg.gpio_num = rx;
  rxcfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rxcfg.resolution_hz = RMT_CLOCK_RESOLUTION;
  rxcfg.mem_block_symbols = 64; // Memory block size
                                // no. of symbols, 64 * 4 = 256 Bytes.
  rxcfg.flags.invert_in = 0;    // Do not invert input
  rxcfg.flags.with_dma = 0;     // No DMA, using CPU memory
  rxcfg.flags.io_loop_back = 0; // no loopback (used for testing)
  rxcfg.intr_priority = 3;      // automatic low-medium priority

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rxcfg, &_rx_channel));

  rmt_rx_event_callbacks_t cbs;
  cbs.on_recv_done = rmt_rx_done_callback;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_rx_channel, &cbs, this));

  // ESP_LOGD(TAG_RMT, "enabling rx channel");
  ESP_ERROR_CHECK(rmt_enable(_rx_channel));
  // ESP_LOGD(TAG_RMT, "enabled rx channel");

  // Set up receive transaction specific parameters.
  // _rvcfg.signal_range_min_ns = 2000;
  // _rvcfg.signal_range_max_ns = MAX_ULTRASONIC_PULSE_NS;
}

void PulseMeasure::init_tx_channel(gpio_num_t tx) {
  //   rmt_tx_channel_config_t txcfg;
  //   memset(&txcfg, 0, sizeof(txcfg));    // avoid bugs from C
  //   txcfg.gpio_num = tx;                 // GPIO number
  //   txcfg.clk_src = RMT_CLK_SRC_DEFAULT; // select source clock
  //   txcfg.resolution_hz = RMT_CLOCK_RESOLUTION;
  //   txcfg.mem_block_symbols = 64; // memory block size, 64 * 4 = 256 Bytes
  //   // TODO: change depth to 1
  //   txcfg.trans_queue_depth = 4;  // number of tx that can pend in the bkgn
  //   txcfg.intr_priority = 3;      // automatic low-med priority
  //   txcfg.flags.invert_out = 0;   // do not invert output signal
  //   txcfg.flags.with_dma = 0;     // do not need DMA backend
  //   txcfg.flags.io_loop_back = 0; // no loopback (used for testing)
  //   txcfg.flags.io_od_mode = 0;   // not open drain GPIO

  //   ESP_ERROR_CHECK(rmt_new_tx_channel(&txcfg, &_tx_channel));

  //   // Set up transmit transaction specific parameters.
  //   // _tvcfg.loop_count = 0;
  //   // _tvcfg.flags.eot_level = 0;         // end of transmission level pin
  //   // low _tvcfg.flags.queue_nonblocking = 0; // block

  //   ESP_ERROR_CHECK(rmt_enable(_tx_channel));

  _tx_pin = tx;
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = BIT64(static_cast<int>(_tx_pin));
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void PulseMeasure::init(gpio_num_t tx, gpio_num_t rx) {
  init_rx_channel(rx);
  init_tx_channel(tx);

  // Initialize copy encoder.
  const rmt_copy_encoder_config_t encfg; // empty struct
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encfg, &_encoder));

  // Initialize the constant tx data.
  _primary_data[0].duration0 = 10;
  _primary_data[0].level0 = 1;

  // The duration of the 2nd pulse is set to 2us to garuntee the 2us clearing
  // time required by the HR-04.
  _primary_data[0].duration1 = 2;
  _primary_data[0].level1 = 0;

  _rx_q = xQueueCreate(5, sizeof(pulse_info_t));
  if (_rx_q == NULL) {
    ESP_LOGE(_TAG, "Could not create rx Queue.");
  }
}

#define DELARE_TIMEIT                                                          \
  int64_t prev = esp_timer_get_time();                                         \
  int64_t t;
#define TIMEIT(x)                                                              \
  t = esp_timer_get_time();                                                    \
  ESP_LOGI(_TAG, "duration " x ": %lld", t - prev);                            \
  prev = t;

/**
 * Call this no faster than...see todo
 */
float PulseMeasure::sample() {
  if (!_pulse_in_flight) {
    _pulse_in_flight = true;
    pinfo.tx_queued = esp_timer_get_time();
    ESP_ERROR_CHECK(rmt_receive(_rx_channel, static_cast<void *>(rxbuf),
                                sizeof(rxbuf), &_rvcfg));

    // ESP_ERROR_CHECK(rmt_transmit(_tx_channel, _encoder, _primary_data,
    //                              sizeof(_primary_data), &_tvcfg));
    portDISABLE_INTERRUPTS();
    gpio_set_level(_tx_pin, 1);
    ets_delay_us(10);
    gpio_set_level(_tx_pin, 0);
    portENABLE_INTERRUPTS();
  }

  pulse_info_t p;
  if (pdTRUE == xQueueReceive(_rx_q, &p, 0)) {
    auto rx_delay = p.rx_done - p.pulse_width - p.tx_queued;
    auto max_delay = p.rx_done - p.tx_queued;
    ESP_LOGI(_TAG, "dTx: %lld", max_delay);
    // ESP_LOGI(_TAG, "dT round trip: %lld", max_delay);
  }

  // Return the last measured distance in cm.
  if (_pulse_duration_us != 0) {
    return _pulse_duration_us * 0.01715f;
  } else {
    return ULTRASONIC_LIMIT_DISTANCE_CM;
  }
}
