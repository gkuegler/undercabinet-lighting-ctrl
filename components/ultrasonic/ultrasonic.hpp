/**
 * Control of HC-SR04.
 */

#include <atomic>

#include "esp_attr.h" // for IRAM_ATTR
#include "freertos/FreeRTOS.h"
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

enum class RangingResult : uint32_t
{
  BAD,
  OKAY
};

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
 * An echo pulse is always returned, even if the object is out of range.
 */
class HCSR04
{
private:
  static const char* tag;
  std::atomic<int> _pulse_in_flight = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  TickType_t _last_range_ticks = 0;
  int _timed_out_ranges = 0;

  TaskHandle_t _task_handle = NULL;
  BaseType_t _task_idx = 0;
  gpio_num_t _trig_pin;
  gpio_num_t _echo_pin;

  // Configurable Params
  static const int _range_timeout_ms = 100;
  static const int _min_pulse_duration_us = 11; // 11us = ~1mm range
  int _delay_between_ranging_ms = 5;

public:
  HCSR04() {};
  ~HCSR04() {};

  // Not actually public.
  static void IRAM_ATTR echo_interrupt_handler(void* pvParam);

  /* Initalize. Call before using object.
  This must be called from the same core that 'taskhandle' runs on.*/
  void init(gpio_num_t trig_pin,
            gpio_num_t echo_pin,
            TaskHandle_t task_to_notify,
            BaseType_t task_idx);

  /* Initial ranging session and wait indefinitely for a valid range. */
  float range_and_wait();

private:
  void trigger_ranging_session();
  void reset();
  constexpr float convert_us_to_range(const uint32_t us);
};
