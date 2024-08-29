#ifndef ST7789_DRIVER_CONFIG_H
#define ST7789_DRIVER_CONFIG_H

#include <driver/spi_common.h>

#define RGB512_BYTES 2

/*** SPI BUS PARAMETERS ***/

#define CONFIG_ST7789_SPI_MODE 0
#define CONFIG_ST7789_SPI_DATA_SETUP_TIME_NS 7
#define CONFIG_ST7789_SPI_HOST SPI3_HOST
#define CONFIG_ST7789_SPI_FLAGS SPICOMMON_BUSFLAG_GPIO_PINS
#define CONFIG_ST7789_DMA_CHANNEL SPI_DMA_CH_AUTO
#define CONFIG_ST7789_CLOCK_SPEED_HZ SPI_MASTER_FREQ_80M

// It is most efficient if the maximum requested transfer size is equal to the
// display buffer size of a gui framwork such as lvgl...
#define CONFIG_ST7789_MAX_TRANSACTION_SZ SPI_MAX_DMA_LEN * 2
#define CONFIG_ST7789_SPI_INTR_FLAGS ESP_INTR_FLAG_LOWMED

// Select cpu core to register SPI ISR.
#define CONFIG_ST7789_SPI_ISR_CPU_ID ESP_INTR_CPU_AFFINITY_AUTO

/*** SPI TRANSACTION PARAMETERS ***/

// LVGL (when double buffered) can only have 2 in flight dma transfers.
#define CONFIG_ST7789_SPI_TRANSACTION_POOL_SIZE 2
// Minimum number of available transactions from the pool before a send is
// allowed.
#define CONFIG_ST7789_SPI_TRANSACTION_POOL_RESERVE 1

// If the port in use implements a port optimised task selection mechanism that
// uses a 'count leading zeros' type instruction (for task selection in a single
// instruction) and configUSE_PORT_OPTIMISED_TASK_SELECTION is set to 1 in
// FreeRTOSConfig.h, then configMAX_PRIORITIES cannot be higher than 32.
#define CONFIG_ST7789_SPI_FLUSH_HANDLER_PRIORITY 5

// While it is not easy to determine how much stack to allocate a task, the RTOS
// does provide functionality that allows a task's stack size to be tuned taking
// a pragmatic trial and error approach; the uxTaskGetStackHighWaterMark() API
// function can be used to see how much stack has actually been used, allowing
// the stack size to be reduced if more was allocated than necessary, and the
// stack overflow detection features can be used to determine if a stack is too
// small.
#define CONFIG_ST7789_SPI_FLUSH_HANDLER_STACK_SIZE configMINIMAL_STACK_SIZE * 3

/*** DISPLAY PARAMETERS ***/

// Mapping for the Adafruit 1.14" 240x135 color TFT
#define CONFIG_DISPLAY_WIDTH_PX 135
#define CONFIG_DISPLAY_HEIGHT_PX 240

// Define where the top left pixel of the display is located in the st7789
// memory.
#define CONFIG_DISPLAY_COL_START_PX 52
#define CONFIG_DISPLAY_ROW_START_PX 40

#define CONFIG_ST7789_MISO_PIN 37
#define CONFIG_ST7789_MOSI_PIN 35
#define CONFIG_ST7789_SCLK_PIN 36
#define CONFIG_ST7789_CS_PIN 7
#define CONFIG_ST7789_DC_PIN 39

#define CONFIG_ST7789_RESET_PIN 40
#define CONFIG_ST7789_POWER_PIN 21
#define CONFIG_ST7789_BACKLIGHT_PIN 45

/* Uncomment to use software reset instead of gpio pin. */
// #define CONFIG_ST7789_SOFT_RST

#endif // ST7789_DRIVER_CONFIG_H