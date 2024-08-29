#ifndef LVGL_DRIVER_ST7789_H
#define LVGL_DRIVER_ST7789_H

#include <stddef.h>
#include <stdint.h>

#include "helper.h"
#include "st7789_config.h"

/**
 * ST7789VW commands
 * Sitronix Datasheet
 * Version 1.0 - 09/2017
 **/
// clang-format off
#define ST7789_NOP          0x00    // Noop
#define ST7789_SWRESET      0x01    // Software reset
#define ST7789_RDDID        0x04    // Read display ID
#define ST7789_RDDST        0x09    // Read display status

#define ST7789_RDDPM        0x0A    // Read display power mode
#define ST7789_RDDMADCTL    0x0B    // Read display MADCTL
#define ST7789_RDDCOLMOD    0x0C    // Read display pixel format
#define ST7789_RDDIM        0x0D    // Read display image mode
#define ST7789_RDDSM        0x0E    // Read display signal mode
#define ST7789_RDDSR        0x0F    // Read display self-diagnostic result (ST7789V)

#define ST7789_SLPIN        0x10    // Sleep in
#define ST7789_SLPOUT       0x11    // Sleep out
#define ST7789_PTLON        0x12    // Personal mode on
#define ST7789_NORON        0x13    // Partial mode off (Normal mode)

#define ST7789_INVOFF       0x20    // Display inversion off
#define ST7789_INVON        0x21    // This play inversion on
#define ST7789_GAMSET       0x26    // Gamma set
#define ST7789_DISPOFF      0x28    // Display off
#define ST7789_DISPON       0x29    // Display on
#define ST7789_CASET        0x2A    // Column address set
#define ST7789_RASET        0x2B    // Row address set
#define ST7789_RAMWR        0x2C    // Memory write
#define ST7789_RAMRD        0x2E    // Memory read

#define ST7789_PTLAR        0x30    // Partial address set
#define ST7789_VSCRDEF      0x33    // Vertical scrolling definition
#define ST7789_TEOFF        0x34    // Tearing effect line off
#define ST7789_TEON         0x35    // Tearing effect line on
#define ST7789_MADCTL       0x36    // Memory data access control
#define ST7789_VSCRSADD     0x37    // Vertical scrolling start address
#define ST7789_IDMOFF       0x38    // Idle mode off
#define ST7789_IDMON        0x39    // Idle mode on
#define ST7789_COLMOD       0x3A    // Interface pixel format
#define ST7789_RAMWRC       0x3C    // Memory write continue
#define ST7789_RAMRDC       0x3E    // Memory read continue

#define ST7789_TESCAN       0x44    // Set tear scanline
#define ST7789_RDTESCAN     0x45    // Get scanline

#define ST7789_WRDISBV      0x51    // Write display brightness
#define ST7789_RDDISBV      0x52    // Read display brightness value
#define ST7789_WRCTRLD      0x53    // Write CTRL display
#define ST7789_RDCTRLD      0x54    // Read CTRL value dsiplay
#define ST7789_WRCACE              0x55    // Write content adaptive brightness control and Color enhancemnet
#define ST7789_RDCABC              0x56    // Read content adaptive brightness control
#define ST7789_WRCABCMB            0x5E    // Write CABC minimum brightness
#define ST7789_RDCABCMB            0x5F    // Read CABC minimum brightness

#define ST7789_RDABCSDR            0x68    // Read Automatic Brightness Control Self-Diagnostic Result
#define ST7789_RDID1               0xDA    // Read ID1
#define ST7789_RDID2               0xDB    // Read ID2
#define ST7789_RDID3               0xDC    // Read ID3

#define COMMAND_DELAY       0xFF    // Add in manual delay

// Note1: In 12-bit/Pixel, 16-bit/Pixel or 18-bit/Pixel mode, the LUT is applied
// to transfer data into the Frame Memory.
// Note2: The Command 3Ah should be set at 55h when writing 16-bit/pixel data
// into frame memory, but 3Ah should be re-set to 66h when reading pixel data
// from frame memory.
#define DATA_COLOR_MODE_RGB565 0x55

// MEMORY DATA ACCESS CONTROL flags
#define MADCTL_TOP_TO_BOTTOM             (0x00 << 7)
#define MADCTL_BOTTOM_TO_TOP             (0x01 << 7)
#define MADCTL_LEFT_TO_RIGHT             (0x00 << 6)
#define MADCTL_RIGHT_TO_LEFT             (0x01 << 6)
#define MADCTL_NORMAL_MODE               (0x00 << 5)
#define MADCTL_REVERSE_MODE              (0x01 << 5)
#define MADCTL_LCD_REFRESH_TOP_TO_BOTTOM (0x00 << 4)
#define MADCTL_LCD_REFRESH_BOTTOM_TO_TOP (0x01 << 4)
#define MADCTL_ORDER_RGB                 (0x00 << 3)
#define MADCTL_ORDER_GBR                 (0x01 << 3)
#define MADCTL_LCD_REFRESH_LEFT_TO_RIGHT (0x00 << 2)
#define MADCTL_LCD_REFRESH_RIGHT_TO_LEFT (0x01 << 2)
// clang-format on

#ifdef __cplusplus
extern "C" {
#endif

/* Represents an area of the screen to be drawn. */
typedef struct {
  int32_t x1;
  int32_t x2;
  int32_t y1;
  int32_t y2;
} st7789_area_t;

void st7789_disp_init();
void st7789_turn_on_backlight();

/* This must be called before writing color data to the screen.*/
void st7789_set_orientation(uint8_t);
/* Send a command byte.*/
void st7789_send_cmd_byte(const uint8_t cmd);
/* Send multiple commands in sequence with no delay.*/
void st7789_send_cmd(const uint8_t *cmd, const size_t length);
void st7789_send_data(const uint8_t *data, const size_t length);
void st7789_flush(const st7789_area_t *area, const uint8_t *color_map);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif // LVGL_DRIVER_ST7789_H