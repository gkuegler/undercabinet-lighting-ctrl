/**
 * Control of HC-SR04.
 */

#include <atomic>
#include <cstdint>

#include "esp_attr.h"     // for IRAM_ATTR
#include "soc/gpio_num.h" // for GPIOs

/*
NOTES ON RANGING:
Max ranging session duration is about ~46,713us
I can take faster samples if I am closer, with a min of 2us before pulse?
I think variable tx excludes the benefit of using rmt tx.
I could still use rmt for precise and continuous recieve.
I can easily handle less than 2us differentiation between two high pulses.
I would need a ring buffer.
I then try to sample at say 100hz, so 10ms between tx attemps.
Every sample call I attempt a tx if no pulse is in flight.
Every sample call I also process any completed samples in rx buffer.
*/

/**
 * HC-SR04 Ultrasonic Distance Sensor from Adafruit
 *
 * Compatible with 3V or 5V.
 *
 * Uses shared GPIO level interrupts.
 *
 * A range is initiated by sending a 10us pulse on trig pin.
 * The unit will then emit a 40kHz burst.
 * The echo pin will return a pulse of a duration equal to the echo time of
 * flight. The unit times out if a return echo is not received. The
 * max time of flight is around ~47ms. When powered by 3v, the unit
 * has difficulty registering small flat objects (such as a hand) after ~150cm.
 *
 * The data sheet for the HC-SR04 shows the echo pulse begins after the
 * reflection is returned, but I have not verified if this is the case so I am
 * assuming the echo pulse can begin as soon as the leading edge of the 10us
 * trigger is sent.
 */
class HCSR04
{
private:
  static const char* TAG;
  int64_t _pulse_start_us = 0;
  int64_t _pulse_duration_us = 0;
  std::atomic<bool> _pulse_in_flight = false;
  std::atomic<bool> _pulse_capture = false;
  gpio_num_t _trig_pin;
  gpio_num_t _echo_pin;
  int _ranging_timeout_start_count = 10; // default
  int _ranging_timeout_samples_remaining =
    _ranging_timeout_start_count; // 100ms default

public:
  HCSR04() {};
  ~HCSR04() {};

  static void IRAM_ATTR echo_interrupt_handler(void* pvParameter);

  /* Initalize. Call before using object. */
  void init(gpio_num_t trig, gpio_num_t echo, int sample_period);

  /**
   * Send a trigger if no pulse is in flight and return the most recently
   * measured range. Call this periodically ideally no faster than the
   * time-of-flight (plus margin) for the desired working distance.
   * Example: 100cm range = 5.83ms pulse, so I could sample
   * every 8.3ms or 120Hz
   */
  float sample();
};
