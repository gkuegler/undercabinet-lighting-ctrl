#include "st7789.h"

void st7789_init_bus_and_display(int orientation) {
  st7789_spi_bus_init();
  st7789_spi_add_device_to_bus();
  st7789_disp_init();
  st7789_set_orientation(orientation);
  st7789_turn_on_backlight();
}