# esp32-st7789-driver
An ST7789 display driver for the ESP32 platform. Provides flush, command, and data write interfaces.

# Using
1. Copy this repository folder into the 'components' folder of an esp32 project.
2. Overide defines in "st7789_config.h" for pin and display configurations specific to your board.
3. Call the high level initilization function 'st7789_init_bus_and_display'.

Note: When using a GUI framework (such as lvgl), you may need to register a callback to the SPI bus to notify when a DMA transfer is complete.
Use 'st7789_spi_register_flush_cb' to register a flush complete callback.

# LVGL Example main.c
```
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Local component includes
#include "st7789.h"

#include "lvgl.h"

static const char* TAG =  "[main]"

// Match the requested SPI transfer size with the desired buffer size.
// This sets how much of the screen can be redrawn at one time.
#define BUF_SIZE CONFIG_ST7789_MAX_TRANSFER_SZ

static lv_obj_t *label;
static lv_display_t *disp;

// STATIC PROTOTYPES
static void guiTask(void *pvParameter);
static void flush_callback();
static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void create_demo_widgets(void);

void app_main(void) {

  // High-level function to initialize the SPI bus and set up the display.
  // Arguments:
  // orientation (int) = A value given 0-3 describing the landscape/portrait
  // rotation of the display. The orientations are display dependent.
  st7789_init_bus_and_display(1);

  // CB notifies lvgl when async dma spi transfer is complete to lvgl can start
  // using the buffer again.
  // Using dependency injection here so there's no st7789 compile dependency on
  // lvgl.
  st7789_spi_register_flush_cb(flush_callback);

  // If you want to use a task to create the graphic, you NEED to create a
  // Pinned task Otherwise there can be problem such as memory corruption and
  // so on. NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to
  // core 0.
  xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);

  // In FreeRTOS, app_main is allowed to return. The task that calls 'app_main'
  //  will delete itself and other tasks will run as normal.
  return;
}

// Creates a semaphore to handle concurrent call to lvgl stuff
//  If you wish to call *any* lvgl function from other threads/tasks
//  you should lock on the very same semaphore.
static SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter) {

  (void)pvParameter;
  xGuiSemaphore = xSemaphoreCreateMutex();

  lv_init();

  // Let lvgl know where to update the takes from.
  // Here I'm using FreeRTOS.
  lv_tick_set_cb(xTaskGetTickCount);

  disp = lv_display_create(CONFIG_DISPLAY_HEIGHT_PX, CONFIG_DISPLAY_WIDTH_PX);

  // Give lvgl our function to write to the display.
  lv_display_set_flush_cb(disp, flush);

  // Initialize the lvgl screen buffers.
  lv_color_t *buf1 = heap_caps_aligned_alloc(4, BUF_SIZE, MALLOC_CAP_DMA);
  assert(buf1 != NULL);

  lv_color_t *buf2 = heap_caps_aligned_alloc(4, BUF_SIZE, MALLOC_CAP_DMA);
  assert(buf2 != NULL);

  lv_display_set_buffers(disp, buf1, buf2, BUF_SIZE,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);

  // Draw the demo widgets to the screen.
  create_demo_widgets();

  while (1) {
    task_delay_ms(10);

    // Try to take the semaphore, call lvgl related function on success.
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
      lv_task_handler();
      xSemaphoreGive(xGuiSemaphore);
    }
  }

  // A task should NEVER return.
  free(buf1);
  free(buf2);
  vTaskDelete(NULL);
}

static void flush_callback() { lv_display_flush_ready(disp); }
static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  UNUSED(disp);
  const st7789_area_t area_wrap = {
      .x1 = area->x1, .x2 = area->x2, .y1 = area->y1, .y2 = area->y2};
  st7789_flush(&area_wrap, (const uint8_t *)px_map);
}

static void create_demo_widgets(void) {
  ESP_LOGD(TAG, "creating demo widgets");
  
  // Change the active screen's background color.
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000),
                            LV_PART_MAIN);

  // Create a white label, set its text and align it to the center.
  label = lv_label_create(lv_screen_active());
  lv_label_set_text_fmt(label, "Hello World!");
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff),
                              LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}
```