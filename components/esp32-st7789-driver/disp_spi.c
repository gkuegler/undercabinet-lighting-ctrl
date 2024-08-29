// FUTURE OPTIMIZATIONS
// Cache Missing?
// The default config puts only the ISR into the IRAM. Other SPI-related
// functions, including the driver itself and the callback, might suffer from
// cache misses and need to wait until the code is read from flash. Select
// CONFIG_SPI_MASTER_IN_IRAM to put the whole SPI driver into IRAM and put the
// entire callback(s) and its callee functions into IRAM to prevent cache
// missing.

#include "disp_spi.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <string.h>

static const char *TAG = "sst7789 spi";

/*
 * Notes About DMA spi_transaction_t Structure Pooling.
 *
 * An xQueue is used to hold a pool of reusable SPI spi_transaction_t
 * structures that get used for all DMA SPI transactions. While an xQueue may
 * seem like overkill it is an already built-in RTOS feature that comes at
 * little cost. xQueues are also ISR safe if it ever becomes necessary to
 * access the pool in the ISR callback.
 *
 * When a DMA request is sent, a transaction structure is removed from the
 * pool, filled out, and passed off to the esp32 SPI driver. Later, when
 * servicing pending SPI transaction results, the transaction structure is
 * recycled back into the pool for later reuse. This matches the DMA SPI
 * transaction life cycle requirements of the esp32 SPI driver.
 *
 * When polling or synchronously sending SPI requests, and as required by the
 * esp32 SPI driver, all pending DMA transactions are first serviced. Then the
 * polling SPI request takes place.
 *
 * When sending an asynchronous DMA SPI request, if the pool is empty, some
 * small percentage of pending transactions are first serviced before sending
 * any new DMA SPI transactions. Not too many and not too few as this balance
 * controls DMA transaction latency.
 *
 * It is therefore not the design that all pending transactions must be
 * serviced and placed back into the pool with DMA SPI requests - that
 * will happen eventually. The pool just needs to contain enough to float some
 * number of in-flight SPI requests to speed up the overall DMA SPI data rate
 * and reduce transaction latency. If however a display driver uses some
 * polling SPI requests or calls disp_wait_for_pending_transactions() directly,
 * the pool will reach the full state more often and speed up DMA queuing.
 */

/* LVGL (when double buffered) can only have 2 in flight dma transfers. */
#define SPI_TRANSACTION_POOL_SIZE 4

/* Minimum number of available transactions from the pool before a send is
 * allowed. */
#define SPI_TRANSACTION_POOL_RESERVE 1

#define RGB512_BYTES 2

/* If the port in use implements a port optimised task selection mechanism that
 * uses a 'count leading zeros' type instruction (for task selection in a single
 * instruction) and configUSE_PORT_OPTIMISED_TASK_SELECTION is set to 1 in
 * FreeRTOSConfig.h, then configMAX_PRIORITIES cannot be higher than 32. */
#define SPI_FLUSH_HANDLER_PRIORITY 5

/* While it is not easy to determine how much stack to allocate a task, the RTOS
 * does provide functionality that allows a task's stack size to be tuned taking
 * a pragmatic trial and error approach; the uxTaskGetStackHighWaterMark() API
 * function can be used to see how much stack has actually been used, allowing
 * the stack size to be reduced if more was allocated than necessary, and the
 * stack overflow detection features can be used to determine if a stack is too
 * small */
#define SPI_FLUSH_HANDLER_STACK_SIZE configMINIMAL_STACK_SIZE * 3

static spi_device_handle_t g_spi;
static QueueHandle_t g_transaction_pool = NULL;
static size_t g_max_transaction_sz = SPI_MAX_TRANSACTION_SIZE;

// The handle of the task that will receive notifications from the
// interrupts. The handle was obtained when the task
// was created.
static TaskHandle_t g_flush_complete_handler;

/* Callback for flush. void* argument should be the lvgl display handle.*/
static flush_cb_t g_flush_cb = NULL;

void st7789_spi_register_flush_cb(flush_cb_t cb) { g_flush_cb = cb; }

/* Handle releasing resources from spi transaction outside of ISR.*/
void spi_release_task(void *pvParameters) {
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(5000);
  static uint32_t ulNotifiedValue = 0;

  for (;;) {
    // Wait to be notified of an interrupt.
    if (xTaskNotifyWait(pdFALSE,          // Don't clear bits on entry.
                        ULONG_MAX,        // Clear all bits on exit.
                        &ulNotifiedValue, // Stores the notified value.
                        xMaxBlockTime) == pdTRUE) {

      if (DISP_SPI_RELEASE_BUS_WHEN_COMPLETE & ulNotifiedValue) {
        spi_device_release_bus(g_spi);
        // ESP_LOGD(TAG, "SPI bus released!");
      }
      if (DISP_SPI_SIGNAL_FLUSH & ulNotifiedValue) {
        if (g_flush_cb) {
          // ESP_LOGD(TAG, "DISP Ready!");
          g_flush_cb();
        } else {
          ESP_LOGD("spi_release_task", "flush finished callback ptr is NULL");
        }
      }
    }
  }
}

/* The interrupt handler does not perform any processing itself. Instead it
 * it unblocks a high priority task in which the events that generated the
 * interrupt are processed. If the priority of the task is high enough then the
 * interrupt will return directly to the task (so it will interrupt one task but
 * return to a different task), so the processing will occur contiguously in
 * time - just as if all the processing had been done in the interrupt handler
 * itself. The status of the interrupting peripheral is sent to the task using
 * an RTOS task notification. */
static void IRAM_ATTR spi_tx_completed_isr(spi_transaction_t *trans) {
  const uint32_t flags = (uint32_t)trans->user;

  // ISR should exit to a higher priority task.
  // The task releasing the spi bus is at a higher priority (5) than
  // 'app_main' (1). This ISR should exit directly to the bus release task.
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;

  if (flags) {
    xTaskNotifyFromISR(g_flush_complete_handler, flags, eSetBits,
                       &xHigherPriorityTaskWoken);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task. The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR().
    I should be causing a context switch to return directly to
    'spi_release_task'. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  return;
}

void st7789_spi_bus_init() {
  ESP_LOGD(TAG, "Initializing bus");

  // Create a task to handle SPI release
  if (pdPASS != xTaskCreate(&spi_release_task, "spi_release_task",
                            SPI_FLUSH_HANDLER_STACK_SIZE, NULL,
                            SPI_FLUSH_HANDLER_PRIORITY,
                            &g_flush_complete_handler)) {
    ESP_LOGE(TAG, "Could not create spi release bus task.");
  }
  if (SPI_MAX_TRANSACTION_SIZE >= CONFIG_ST7789_MAX_TRANSACTION_SZ) {
    g_max_transaction_sz = CONFIG_ST7789_MAX_TRANSACTION_SZ;
  } else {
    ESP_LOGW(TAG,
             "Max transaction size requested was greater than available "
             "on this platform. Defaulting to %d",
             SPI_MAX_TRANSACTION_SIZE);
  }

  spi_bus_config_t buscfg = {.mosi_io_num = CONFIG_ST7789_MOSI_PIN,
                             .miso_io_num = CONFIG_ST7789_MISO_PIN,
                             .sclk_io_num = CONFIG_ST7789_SCLK_PIN,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .data4_io_num = -1,
                             .data5_io_num = -1,
                             .data6_io_num = -1,
                             .data7_io_num = -1,
                             .max_transfer_sz = g_max_transaction_sz,
                             .flags = SPICOMMON_BUSFLAG_MASTER |
                                      SPICOMMON_BUSFLAG_GPIO_PINS,
                             // Select cpu core to register SPI ISR.
                             .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                             /*< Interrupt flag for the bus to set the
                              * priority, and IRAM attribute, see
                              *  ``esp_intr_alloc.h``. Note that the EDGE,
                              * INTRDISABLED attribute are ignored by the
                              * driver. Note that if ESP_INTR_FLAG_IRAM is set,
                              * ALL the callbacks of the driver, and their
                              * callee functions, should be put in the IRAM.
                              */
                             .intr_flags = ESP_INTR_FLAG_LOWMED};

  ESP_LOGD(TAG,
           "BUS CONFIGURATION ->\nmosi_io_num: %d\nmiso_io_num: "
           "%d\nsclk_io_num: %d\nmax_transfer_sz: %d\n",
           buscfg.mosi_io_num, buscfg.miso_io_num, buscfg.sclk_io_num,
           buscfg.max_transfer_sz);

  ESP_ERROR_CHECK(
      spi_bus_initialize(SPI_HOST_TFT, &buscfg, CONFIG_ST7789_DMA_CHANNEL));
}

void populate_transaction_pool() {
  if (g_transaction_pool == NULL) {
    g_transaction_pool =
        xQueueCreate(SPI_TRANSACTION_POOL_SIZE, sizeof(spi_transaction_t *));
    assert(g_transaction_pool != NULL);
    for (size_t i = 0; i < SPI_TRANSACTION_POOL_SIZE; i++) {
      spi_transaction_t *ptrx = (spi_transaction_t *)heap_caps_malloc(
          sizeof(spi_transaction_t), MALLOC_CAP_DMA);
      assert(ptrx != NULL);
      memset(ptrx, 0, sizeof(spi_transaction_t));
      xQueueSend(g_transaction_pool, &ptrx, portMAX_DELAY);
    }
  }
};

void st7789_spi_add_device_to_bus() {
  spi_device_interface_config_t devcfg = {
      .command_bits = 0,
      .address_bits = 0,
      .clock_speed_hz = CONFIG_ST7789_CLOCK_SPEED_HZ,
      .mode = SPI_TFT_SPI_MODE,
      .spics_io_num = CONFIG_ST7789_CS_PIN,
      // .input_delay_ns = ST7789_SPI_DATA_SETUP_TIME_NS,
      .input_delay_ns = 0,
      .queue_size = SPI_TRANSACTION_POOL_SIZE,
      .pre_cb = NULL,
      .post_cb = spi_tx_completed_isr,
      /* SPI_DEVICE_NO_DUMMY: dummy bits are not required with half duplex (when
       * only writing to the controller). Using half duplex and disabling dummy
       * bits means that 80MHz sclk can freq can be used even with gpio pins
       * instead of IOMUX_PINS */
      .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX,
  };

  ESP_LOGD(TAG,
           "DEVICE INTERFACE CONFIGURATION ->\n  clock_speed_hz: %d\n  mode: "
           "%" PRIu8
           "\n  spics_io_num: %d\n  input_delay_ns: %d\n  queue_size: %d\n"
           "flags: %" PRIu32 "\n",
           devcfg.clock_speed_hz, devcfg.mode, devcfg.spics_io_num,
           devcfg.input_delay_ns, devcfg.queue_size, devcfg.flags);

  ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_ST7789_SPI_HOST, &devcfg, &g_spi));

  populate_transaction_pool();
}

void wait_for_pool_availibility(size_t count) {
  spi_transaction_t *presult;
  while (uxQueueMessagesWaiting(g_transaction_pool) < count) {
    if (spi_device_get_trans_result(g_spi, &presult, 1) == ESP_OK) {
      // Return transaction structure to the pool.
      xQueueSend(g_transaction_pool, &presult, portMAX_DELAY);
    }
  }
}

/* Wait for all async transactions to finish. */
void disp_wait_for_pending_transactions(void) {
  wait_for_pool_availibility(SPI_TRANSACTION_POOL_SIZE);
}

void disp_spi_transaction_raw(const uint8_t *data, size_t length,
                              disp_spi_send_flag_t userflags,
                              uint32_t tx_flags) {
  assert(0 < length);

  spi_transaction_t tx = {0};
  tx.flags |= tx_flags;

  // Save flags for pre/post transaction processing.
  tx.user = (void *)userflags;

  // Not receiving data.
  tx.rx_buffer = NULL;

  // Transaction length is in bits.
  tx.length = length * 8;

  // Store the tx data in the 32bit pointer if possible.
  //  This is much faster forsending 4 bytes worth of data, such as commands.
  if (length <= 4 && data != NULL) {
    tx.flags |= SPI_TRANS_USE_TXDATA;
    memcpy(tx.tx_data, data, length);
  } else {
    tx.tx_buffer = data;
  }

  if (userflags & DISP_SPI_SEND_POLLING) {
    // Before polling, all previous pending transactions need to be serviced.
    disp_wait_for_pending_transactions();
    spi_device_polling_transmit(g_spi, (spi_transaction_t *)&tx);
  } else if (userflags & DISP_SPI_SEND_SYNCHRONOUS) {
    // Before synchronous queueing, all previous pending transactions need to
    // be serviced.
    disp_wait_for_pending_transactions();
    spi_device_transmit(g_spi, (spi_transaction_t *)&tx);
  } else {
    // If necessary, ensure we can queue new transactions by servicing some
    // previous transactions.
    if (uxQueueMessagesWaiting(g_transaction_pool) == 0) {
      wait_for_pool_availibility(SPI_TRANSACTION_POOL_RESERVE);
    }

    spi_transaction_t *pPoolTx = NULL;

    // Pull an allocated transaction structure from the pool and queue to spi
    // bus.
    xQueueReceive(g_transaction_pool, &pPoolTx, portMAX_DELAY);
    memcpy(pPoolTx, &tx, sizeof(tx));
    if (spi_device_queue_trans(g_spi, pPoolTx, portMAX_DELAY) != ESP_OK) {
      // Send failed transaction back to the pool to be reused.
      xQueueSend(g_transaction_pool, &pPoolTx, portMAX_DELAY);
    }
  }
}

/**
 * 'userflags' denotes what type of transaction to send.
 * Transactions above 'SPI_MAX_TRANSACTION_SIZE' can't be polling or async.
 */
void disp_spi_transaction(const uint8_t *data, size_t length,
                          disp_spi_send_flag_t userflags) {
  /* Split up data over multiple transactions.*/
  if (g_max_transaction_sz < length) {
    size_t idx = 0;
    const size_t chunk_size = SPI_MAX_DMA_LEN;

    // Acquire the bus to keep cs active and minimally speed up transfers.
    // Must set 'wait' to 'portMAX_DELAY' according to docs.
    ESP_ERROR_CHECK(spi_device_acquire_bus(g_spi, portMAX_DELAY));

    while (g_max_transaction_sz < length - idx) {
      disp_spi_transaction_raw(data + idx, chunk_size, userflags,
                               SPI_TRANS_CS_KEEP_ACTIVE);
      idx += chunk_size;
    }

    // Final transaction releases the bus on its callback.
    disp_spi_transaction_raw(data + idx, length - idx,
                             userflags | DISP_SPI_RELEASE_BUS_WHEN_COMPLETE, 0);

  } else {
    disp_spi_transaction_raw(data, length, userflags, 0);
  }
}
