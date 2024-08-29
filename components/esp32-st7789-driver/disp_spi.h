#ifndef ST7789_DISP_SPI_H
#define ST7789_DISP_SPI_H

#include "st7789_config.h"

#include <driver/spi_master.h>

// PRIu8
// PRIu16
// PRIu32
// PRIu64
#include <stdint.h>

// Max transaction send buffer size as defined by ESP32S3 docs.
// DMA transfer block sizes will be smaller than this limit, but the ESP32 spi
// driver sets up the linked list of DMA blocks for us.
// TODO: make this a check for different hardware
#define SPI_MAX_TRANSACTION_SIZE 32768

// Available ESP32S3 clock spi clock frequencies.
#define SPI_MASTER_FREQ_8M (80 * 1000 * 1000 / 10) ///< 8MHz
#define SPI_MASTER_FREQ_9M (80 * 1000 * 1000 / 9)  ///< 8.89MHz
#define SPI_MASTER_FREQ_10M (80 * 1000 * 1000 / 8) ///< 10MHz
#define SPI_MASTER_FREQ_11M (80 * 1000 * 1000 / 7) ///< 11.43MHz
#define SPI_MASTER_FREQ_13M (80 * 1000 * 1000 / 6) ///< 13.33MHz
#define SPI_MASTER_FREQ_16M (80 * 1000 * 1000 / 5) ///< 16MHz
#define SPI_MASTER_FREQ_20M (80 * 1000 * 1000 / 4) ///< 20MHz
#define SPI_MASTER_FREQ_26M (80 * 1000 * 1000 / 3) ///< 26.67MHz
#define SPI_MASTER_FREQ_40M (80 * 1000 * 1000 / 2) ///< 40MHz
// TODO: there may be a warning with using clock frequencies above 40MHz for the
// st7789
#define SPI_MASTER_FREQ_80M (80 * 1000 * 1000 / 1) ///< 80MHz

#define SPI_TFT_SPI_MODE (0)
#define ST7789_SPI_DATA_SETUP_TIME_NS 7
#define SPI_HOST_TFT SPI3_HOST

#ifdef __cplusplus
extern "C" {
#endif
typedef enum _disp_spi_send_flag_t {
  DISP_SPI_SEND_QUEUED = 1 << 0,
  DISP_SPI_SEND_POLLING = 1 << 1,
  DISP_SPI_SEND_SYNCHRONOUS = 1 << 2,
  DISP_SPI_SIGNAL_FLUSH = 1 << 3,
  DISP_SPI_RELEASE_BUS_WHEN_COMPLETE = 1 << 4,
} disp_spi_send_flag_t;

/* Callback to call when flush is completed.*/
typedef void (*flush_cb_t)();

void st7789_spi_register_flush_cb(flush_cb_t flush_cb);
void st7789_spi_bus_init();
void st7789_spi_add_device_to_bus();
// TODO: implement
// disp_spi_remove_device_from_bus();

/** Important!
 *  All buffers should also be 32-bit aligned and DMA capable to prevent
 *  extra allocations and copying. When DMA reading (even in polling mode) the
 *  ESP32 always read in 4-byte chunks even if less is requested. Extra space
 *  will be zero filled. Always ensure the out buffer is large enough to hold at
 *  least 4 bytes!
 */
void disp_spi_transaction(const uint8_t *data, size_t length,
                          disp_spi_send_flag_t flags);

void disp_wait_for_pending_transactions(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ST7789_DISP_SPI_H*/
