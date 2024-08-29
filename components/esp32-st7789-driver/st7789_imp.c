#include "st7789.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "disp_spi.h"
#include "helper.h"

static const char *TAG = "st7789";

    typedef struct {
  uint16_t col_start;
  uint16_t col_end;
  uint16_t row_start;
  uint16_t row_end;
} display_rect_t;

#define ST7789_MEMORY_WIDTH_PX 240
#define ST7789_MEMORY_HEIGHT_PX 320

// This is where the screen is mapped to the ST7789's memory. (col_start,
// y0) is top left of the screen in normal orientation.
static display_rect_t g_display_window;

// The LCD needs a bunch of command/argument values to be initialized. They are
// stored in this struct.
typedef struct {
  uint8_t cmd;
  uint8_t data[4];
  uint8_t data_length; // No of data in data; bit 7 = delay after set; 0xFF =
                       // end of cmds.
} lcd_init_cmd_t;

static lcd_init_cmd_t st7789_init_cmds[] = {
    {ST7789_SLPOUT, {0}, 0},
    {COMMAND_DELAY, {0}, 150},
    {ST7789_COLMOD, {DATA_COLOR_MODE_RGB565}, 1},
    {COMMAND_DELAY, {0}, 10},

    // Default to portrait mode.
    {ST7789_MADCTL,
     {MADCTL_TOP_TO_BOTTOM | MADCTL_LEFT_TO_RIGHT | MADCTL_NORMAL_MODE |
      MADCTL_LCD_REFRESH_TOP_TO_BOTTOM | MADCTL_ORDER_RGB |
      MADCTL_LCD_REFRESH_LEFT_TO_RIGHT},
     1},

    // The ST7789 accepts data MSB first.
    // ESP32 is little endian so LSB is sent first from arrays.
    // The Adafruit TFT uses 16 bit color data which will end up in the wrong
    // order. This is a way to invert colors in the hardware so they come out
    // correct.
    // I saw this used by adafruit themselves.
    // https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp
    {ST7789_INVON, {0}, 0},

    {COMMAND_DELAY, {0}, 10},
    {ST7789_NORON, {0}, 0},
    {COMMAND_DELAY, {0}, 10},
    {ST7789_DISPON, {0}, 0},
};

void st7789_send_cmd_byte(const uint8_t cmd) {
  static uint8_t cmdbuf = 0;
  cmdbuf = cmd;
  st7789_send_cmd(&cmdbuf, 1);
}

void st7789_send_cmd(const uint8_t *cmd, const size_t length) {
  disp_wait_for_pending_transactions();
  gpio_set_level(CONFIG_ST7789_DC_PIN, 0);
  disp_spi_transaction(cmd, length, DISP_SPI_SEND_SYNCHRONOUS);
}

void st7789_send_data(const uint8_t *data, const size_t length) {
  disp_wait_for_pending_transactions();
  gpio_set_level(CONFIG_ST7789_DC_PIN, 1);
  disp_spi_transaction(data, length, DISP_SPI_SEND_SYNCHRONOUS);
}

// static void st7789_send_color(void *data, size_t length) {
void st7789_send_color(const uint8_t *data, const size_t length) {
  disp_wait_for_pending_transactions();
  gpio_set_level(CONFIG_ST7789_DC_PIN, 1);
  disp_spi_transaction(data, length,
                       DISP_SPI_SEND_QUEUED | DISP_SPI_SIGNAL_FLUSH);
}

void st7789_set_orientation(uint8_t orientation) {

  uint8_t madctl = MADCTL_LCD_REFRESH_TOP_TO_BOTTOM | MADCTL_ORDER_RGB |
                   MADCTL_LCD_REFRESH_LEFT_TO_RIGHT;

  // TODO: optimize this?
  const uint16_t bottom_index = ST7789_MEMORY_HEIGHT_PX -
                                CONFIG_DISPLAY_ROW_START_PX -
                                CONFIG_DISPLAY_HEIGHT_PX;
  const uint16_t right_index = ST7789_MEMORY_WIDTH_PX -
                               CONFIG_DISPLAY_COL_START_PX -
                               CONFIG_DISPLAY_WIDTH_PX;

  switch (orientation) {
  case 0:
    madctl |= MADCTL_TOP_TO_BOTTOM | MADCTL_LEFT_TO_RIGHT | MADCTL_NORMAL_MODE;
    g_display_window.col_start = CONFIG_DISPLAY_COL_START_PX;
    g_display_window.col_end =
        CONFIG_DISPLAY_COL_START_PX + CONFIG_DISPLAY_WIDTH_PX - 1;
    g_display_window.row_start = CONFIG_DISPLAY_ROW_START_PX;
    g_display_window.row_end =
        CONFIG_DISPLAY_ROW_START_PX + CONFIG_DISPLAY_HEIGHT_PX - 1;
    break;
  case 1:
    madctl |= MADCTL_BOTTOM_TO_TOP | MADCTL_LEFT_TO_RIGHT | MADCTL_REVERSE_MODE;
    // Note that the coordinates for row and column addressing change now to
    // (0,0) being in the bottom left corner. The memory map window remains the
    // same but new start and stop values must be calculated from the existing
    // window using only the supplied top and left padding values. Column start
    // is now at the hardware bottom.
    g_display_window.col_start = bottom_index;
    g_display_window.col_end = bottom_index + CONFIG_DISPLAY_HEIGHT_PX - 1;
    g_display_window.row_start = CONFIG_DISPLAY_COL_START_PX;
    g_display_window.row_end =
        CONFIG_DISPLAY_COL_START_PX + CONFIG_DISPLAY_WIDTH_PX - 1;
    break;
  case 2:
    madctl |= MADCTL_BOTTOM_TO_TOP | MADCTL_RIGHT_TO_LEFT | MADCTL_NORMAL_MODE;
    g_display_window.col_start = right_index;
    g_display_window.col_end = right_index + CONFIG_DISPLAY_WIDTH_PX - 1;
    g_display_window.row_start = bottom_index;
    g_display_window.row_end = bottom_index + CONFIG_DISPLAY_HEIGHT_PX - 1;
    break;
  case 3:
    madctl |= MADCTL_TOP_TO_BOTTOM | MADCTL_RIGHT_TO_LEFT | MADCTL_REVERSE_MODE;
    g_display_window.col_start = CONFIG_DISPLAY_ROW_START_PX;
    g_display_window.col_end =
        CONFIG_DISPLAY_ROW_START_PX + CONFIG_DISPLAY_WIDTH_PX - 1;
    g_display_window.row_start = right_index;
    g_display_window.row_end = right_index + CONFIG_DISPLAY_HEIGHT_PX - 1;
    break;
  default:
    ESP_LOGW(
        TAG,
        "An orientation value of '%d' is not allowed. Alowed values are 0-3.",
        orientation);
    break;
  }

  st7789_send_cmd_byte(ST7789_MADCTL);
  st7789_send_data((void *)&madctl, 1);
}

uint16_t swap_byte_order(uint16_t n) { return (n >> 8) | (n << 8); }

void st7789_set_address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  // note: high byte and low byte get sent seperately in big endian
  static uint16_t col_bounds[2];
  static uint16_t row_bounds[2];

  col_bounds[0] = swap_byte_order(g_display_window.col_start + x1);
  col_bounds[1] = swap_byte_order(g_display_window.col_start + x2);
  row_bounds[0] = swap_byte_order(g_display_window.row_start + y1);
  row_bounds[1] = swap_byte_order(g_display_window.row_start + y2);

  st7789_send_cmd_byte(ST7789_CASET);
  st7789_send_data((void *)&col_bounds, 4);

  st7789_send_cmd_byte(ST7789_RASET);
  st7789_send_data((void *)&row_bounds, 4);
}

// Reset the display to a known state.
void st7789_reset() {
  ESP_LOGD(TAG, "Resetting ST7789.");
#ifndef CONFIG_ST7789_SOFT_RST
  gpio_set_level((gpio_num_t)CONFIG_ST7789_RESET_PIN, 0);
  task_delay_ms(100);
  gpio_set_level((gpio_num_t)CONFIG_ST7789_RESET_PIN, 1);
  task_delay_ms(250);
#else
  delay_ms(100);
  st7789_send_cmd_byte(ST7789_SWRESET);
  delay_ms(250);
#endif
}

void st7789_disp_init() {
  ESP_LOGD(TAG, "Initalizing the ST7789 spi chip.");

  gpio_config_t cfg_pwr_pin = {
      .pin_bit_mask = BIT64(CONFIG_ST7789_POWER_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = true,
      .pull_down_en = false,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&cfg_pwr_pin);

  // Avoid problems with relying on RTC GPIO pullups.
  gpio_set_level((gpio_num_t)CONFIG_ST7789_POWER_PIN, 1);
  task_delay_ms(300);

#ifndef CONFIG_ST7789_SOFT_RST
  gpio_reset_pin((gpio_num_t)CONFIG_ST7789_RESET_PIN);
  gpio_set_direction((gpio_num_t)CONFIG_ST7789_RESET_PIN, GPIO_MODE_OUTPUT);
#endif

  st7789_reset();

  gpio_reset_pin((gpio_num_t)CONFIG_ST7789_DC_PIN);
  gpio_set_direction((gpio_num_t)CONFIG_ST7789_DC_PIN, GPIO_MODE_OUTPUT);

  ESP_LOGD(TAG, "Sending init commands.");

  for (size_t i = 0; i < ARRAY_LENGTH(st7789_init_cmds); i++) {
    lcd_init_cmd_t cmd = st7789_init_cmds[i];
    if (COMMAND_DELAY == cmd.cmd) {
      task_delay_ms(cmd.data_length);
    } else {
      st7789_send_cmd_byte(cmd.cmd);
      if (0 < cmd.data_length) {
        st7789_send_data(cmd.data, cmd.data_length);
      }
    }
  }

  ESP_LOGD(TAG, "Initialized.");
}

void st7789_flush(const st7789_area_t *area, const uint8_t *color_map) {
  // set col and row modes
  st7789_set_address(area->x1, area->y1, area->x2, area->y2);

  const size_t length =
      (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2;

  st7789_send_cmd_byte(ST7789_RAMWR);
  st7789_send_color(color_map, length);
}

void st7789_turn_on_backlight() {
  gpio_reset_pin(CONFIG_ST7789_BACKLIGHT_PIN);
  gpio_set_direction(CONFIG_ST7789_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(CONFIG_ST7789_BACKLIGHT_PIN, 1);
  ESP_LOGD(TAG, "Backlight On.");
}
