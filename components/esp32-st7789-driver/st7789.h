#ifndef ST7789_H
#define ST7789_H

#include "disp_spi.h"
#include "st7789_config.h"
#include "st7789_imp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* High-level function to initialize the SPI bus and set up the display.
 * Arguments:
 * orientation (int) = A value given 0-3 describing the landscape/portrait
 * rotation of the display. The orientations are display dependent. */
void st7789_init_bus_and_display(int orientation);

#ifdef __cplusplus
}
#endif

#endif // ST7789_H