#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "inttypes.h"

#define UART_PORT_NUM UART_NUM_1
#define UART_TX_PIN   GPIO_NUM_17
#define UART_RX_PIN   GPIO_NUM_18
struct __attribute__((__packed__)) UartMessage
{
  uint8_t code;
  // use this to determine if my full message was sent?
  uint8_t nbData;
  void* data;
};

static void
test_uart_task(void* pvParameter)
{
  static const size_t rxbufsize = 128;
  static uint8_t rxbuf[rxbufsize];

  uart_config_t uart_config = {
    .baud_rate = TEST_UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
  };
  uart_driver_install(TEST_UART_PORT_NUM, 256, 0, 0, NULL, 0);
  uart_param_config(TEST_UART_PORT_NUM, &uart_config);
  uart_set_pin(TEST_UART_PORT_NUM,
               TEST_UART_TX_PIN,
               TEST_UART_RX_PIN,
               UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  while (true) {
    memset(rxbuf, 0, rxbufsize);
    int len = uart_read_bytes(TEST_UART_PORT_NUM, rxbuf, sizeof(rxbuf), 0);
    if (len > 0 && len < rxbufsize) {
      ESP_LOGD("uart", "received '%d' bytes", len);
      if (strcmp("cal", reinterpret_cast<const char*>(rxbuf)) == 0) {
        eventq.send(Event::START_SET_THRESHOLD_DISTANCE);
      } else if (strcmp("pulse", reinterpret_cast<const char*>(rxbuf)) == 0) {
        eventq.send(Event::CHIRP);
      }

      uart_write_bytes(TEST_UART_PORT_NUM, (const char*)rxbuf, len);
      uart_flush(UART_NUM_1);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
