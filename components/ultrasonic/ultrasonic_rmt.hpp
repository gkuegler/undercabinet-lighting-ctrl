/*
Use at least C++20.
ESP-IDF uses GNU++23 by default.

This has been superceded by the regular ultrasonic.hpp class.
RMT has not been that effective for single pulse measuring.
Interrupt based flipflops have proved faster.

Downside of RMT:
Can't set max duration for 1's or 0's. 'signal_range_max_ns' ends the
transaction when reached on a 1 or 0. Once the gpio level goes to 0 (after the
echo pulse), rmt waits for 'signal_range_max_ns' until the interrupt is
generated.

This means I always have to wait for at least the longest expected pulse length
after the echo goes to 0 before the rx is completed and I can be certain the
HC-SR04 is ready to trigger again. The timout on my HC-SR04 from adafuit appears
to be about ~47ms. This means if nothing is in front of the sensor, the echo
pulse will last for ~47ms.

If I set the 'signal_range_max_ns' to a reasonable amount (8.7ms => 150cm range)
and try to sample say every 10ms, the rx will timout on the high pulse and the
echo line could remain high when I initiate an new rx.

If I use interrupt based stops, I can measure exactly when the echo pulse stops
regardless of how long it is. I can take advantage of short pulses to increase
sample speed. If my working distances correspond to an echo puse length of 5ms,
I can attempt to trigger say every 10ms (100hz). I skip attempts until the pulse
times out. This gives me safety on long measurements while enabling me to take
advance of a faster fixed sample rate when the ranged distances falls below a
certain threshold.
*/

#include <assert.h>
#include <atomic>
#include <stdint.h>
#include <string_view>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// https://stackoverflow.com/questions/19532826/what-does-a-dangerous-relocation-error-mean

// TODO: make it clear the relationship between clock rate, max distance
// measurement, and maximum sample rates/minimum sample interval

/*
The duration of a pulse is counted in clock ticks, and the number of ticks of a
pulse has to fit into a 'uint32_t'. This means if I want to measure long pulses
I have to use a slower clock rate.
*/
#define ULTRASONIC_LIMIT_DISTANCE_CM 100.0

struct pulse_info_t {
  int64_t tx_queued;
  uint16_t pulse_width;
  int64_t rx_done;
};

class PulseMeasure {
private:
  // TODO: make this calculation depend on my desired maximum distance an pick a
  // target clock?.
  // 1 MHz resolution, 1 us per tick
  // 1us = 0.0343cm abs dist, so +- 0.01715cm on my dist down & back
  static constexpr uint32_t RMT_CLOCK_RESOLUTION = (1 * 1000 * 1000);

  constexpr uint32_t rmt_us_to_ticks(uint32_t us) {
    return (RMT_CLOCK_RESOLUTION / (1 * 1000 * 1000)) * us;
  }

  // Conversion from time duration to target distance.
  // (34300 cm/s) * (1/1000000000 s/ns) * 0.5
  // (accounts for down and back)
  // Max value of uint32_t => 4,294,967,295
  static constexpr uint32_t MAX_ULTRASONIC_PULSE_NS =
      (ULTRASONIC_LIMIT_DISTANCE_CM / 34300.0) * (1 * 1000 * 1000 * 1000) * 2;

  static_assert(MAX_ULTRASONIC_PULSE_NS > sizeof(uint32_t),
                "Maximum ultrasonic distances to large for RMT.");

  // The handle of the task that will receive notifications from the
  // rx finished interrupt routine.
  QueueHandle_t _rx_q;

  gpio_num_t _tx_pin;
  gpio_num_t _rx_pin;
  rmt_channel_handle_t _rx_channel = NULL;
  rmt_channel_handle_t _tx_channel = NULL;

  // Set up receive transaction specific parameters.
  const rmt_receive_config_t _rvcfg = {
      .signal_range_min_ns = 2000, // ignore pulses shorter than
      .signal_range_max_ns = MAX_ULTRASONIC_PULSE_NS // stop
  };

  // Set up transmit transaction specific parameters.
  const rmt_transmit_config_t _tvcfg = {
      .loop_count = 0, // don't loop
      .flags = {
          .eot_level = 0,        // end of transmission level pin low
          .queue_nonblocking = 0 // block
      }};

  rmt_encoder_handle_t _encoder = NULL;
  rmt_symbol_word_t _primary_data[1];

  // TODO: Correlate this to the rx memory block symbol count.
  uint8_t rxbuf[256];

  // The last measured distance.
  std::atomic<bool> _pulse_in_flight = false;
  std::atomic<uint16_t> _pulse_duration_us = 0;
  pulse_info_t pinfo;

public:
  PulseMeasure(){};
  ~PulseMeasure(){};

  // IRAM attribute causing linking error: dangerous relocation: l32r: literal
  // placed after use static bool IRAM_ATTR rmt_rx_done_callback(
  static bool IRAM_ATTR rmt_rx_done_callback(
      rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data,
      void *user_ctx);

  void init_rx_channel(gpio_num_t rx);
  void init_tx_channel(gpio_num_t tx);
  void init(gpio_num_t tx, gpio_num_t rx);

  /**
   * Call this no faster than...see todo
   */
  float sample();
};
