/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief esp-idf portability implementation
 *  \version 2.0
 */
#include "sys/sys_esp.h"
#include "sys.h"
#include "helpers.h"
#include <stdlib.h>  /* malloc, free */
#include <string.h>  /* memcpy */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"

/* ----------------------------------------------------------------------------------------------
 * i2c
 */

typedef struct
{
   i2c_lowlevel_config config;
   i2c_master_bus_handle_t bus;
   bool bus_created;
   i2c_master_dev_handle_t device;
   uint32_t timeout;
} esp_i2c_t;

i2c_lowlevel_context SYS_WEAK i2c_ll_init(uint8_t i2c_address, uint32_t i2c_speed, uint32_t i2c_timeout_ms,
                                      i2c_lowlevel_config *config)
{
   i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = i2c_address,
      .scl_speed_hz = i2c_speed,
   };

   esp_i2c_t *l = (esp_i2c_t *) calloc(1, sizeof *l);
   if(NULL == l)
      return NULL; 
   memcpy(&l->config, config, sizeof l->config);
   l->timeout = i2c_timeout_ms;

   if(NULL == config->bus)
   {
      i2c_master_bus_config_t bus_cfg = {
         .clk_source = I2C_CLK_SRC_DEFAULT,
         .i2c_port = config->port,
         .sda_io_num = config->pin_sda,
         .scl_io_num = config->pin_scl,
         .glitch_ignore_cnt = 7,
         .flags.enable_internal_pullup = true,
      };
      if(i2c_new_master_bus(&bus_cfg, &l->bus) != ESP_OK)
      {
         SERR("Failed to initialize I2C bus");
         free(l);
         return NULL;
      }
      l->config.bus = &l->bus;
      l->bus_created = true;
   }
   else
      l->bus_created = false;

   if(i2c_master_bus_add_device(*l->config.bus, &dev_cfg, &l->device) != ESP_OK)
   {
      SERR("I2C initialization failed");
      free(l);
      return NULL;
   }

   return (i2c_lowlevel_context) l;
}

bool SYS_WEAK i2c_ll_deinit(i2c_lowlevel_context ctx)
{
   esp_i2c_t *l = (esp_i2c_t *) ctx;
   if(l->bus_created)
      i2c_del_master_bus(l->bus);
   free(l);
   return true;
}

bool SYS_WEAK i2c_ll_write(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length)
{
   esp_i2c_t *l = (esp_i2c_t *) ctx;
   return (i2c_master_transmit(l->device, data, length, -1) == ESP_OK);
}

bool SYS_WEAK i2c_ll_write_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length)
{
   esp_i2c_t *l = (esp_i2c_t *) ctx;
   uint8_t *buffer;
   esp_err_t result;

   buffer = (uint8_t *) malloc(length + 1);
   if(NULL == buffer)
      return false;

   buffer[0] = reg;
   memcpy(&buffer[1], data, length);
   result = i2c_master_transmit(l->device, buffer, length+1, -1);
   free(buffer);

   return (result == ESP_OK);
}

bool SYS_WEAK i2c_ll_read(i2c_lowlevel_context ctx, uint8_t *data, uint8_t length)
{
   esp_i2c_t *l = (esp_i2c_t *) ctx;
   return (i2c_master_receive(l->device, data, length, -1) == ESP_OK);
}

bool SYS_WEAK i2c_ll_read_reg(i2c_lowlevel_context ctx, uint8_t reg, uint8_t *data, uint8_t length)
{
   esp_i2c_t *l = (esp_i2c_t *) ctx;
   return (i2c_master_transmit_receive(l->device, &reg, 1, data, length, -1) == ESP_OK);
}

/* ----------------------------------------------------------------------------------------------
 * uart
 */

#include <driver/uart.h>

#define SYS_UART_EVENT_QUEUE_DEPTH 10

typedef struct esp_uart_s
{
   uart_port_t port;
   QueueHandle_t events;
   uart_ll_rx_handler_fn rx_handler;
   void *rx_cookie;
} esp_uart_t;

/* Received data is handed over one idle-delimited burst at a time.

   The obvious implementation -- read a fixed number of bytes in a loop -- does
   not work for a protocol whose messages are delimited by silence, because the
   caller's parser is told when a *callback* happened and not when each byte
   arrived. Fixed-size reads put the boundaries wherever the driver split the
   stream, and for a message size that divides the read size the split stays in
   the same wrong place for as long as the link is up. Waiting on the driver's
   idle timeout instead means one callback per burst, so the gaps the parser
   sees are the gaps that were really on the wire. */
static void uart_ll_rx_task(void *param)
{
   esp_uart_t *l = (esp_uart_t *) param;
   uint8_t buffer[SYS_UART_DEFAULT_RX_BUFFER];
   uart_event_t event;

   for(;;)
   {
      if(xQueueReceive(l->events, &event, portMAX_DELAY) != pdTRUE)
         continue;

      switch(event.type)
      {
         case UART_DATA:
         {
            size_t remaining = event.size;
            while(remaining > 0)
            {
               size_t want = (remaining > sizeof buffer) ? sizeof buffer : remaining;
               int got = uart_read_bytes(l->port, buffer, want, 0);
               if(got <= 0)
                  break;
               remaining -= (size_t) got;
               if(NULL != l->rx_handler)
                  l->rx_handler(buffer, (uint32_t) got, l->rx_cookie);
            }
            break;
         }

         case UART_FIFO_OVF:
         case UART_BUFFER_FULL:
            /* Whatever is in the buffer is of unknown alignment now. Dropping
               it costs one message; keeping it costs a parser resynchronisation
               against data that never made sense. */
            uart_flush_input(l->port);
            xQueueReset(l->events);
            break;

         default:
            break;
      }
   }
}

uart_lowlevel_context SYS_WEAK uart_ll_init(uint32_t baud, uart_ll_data_bits data_bits, uart_ll_stop_bits stop_bits,
                                            uart_ll_parity parity, uart_lowlevel_config *config)
{
   esp_uart_t *l;
   uart_config_t uart_config = {
      .baud_rate  = baud,
      .parity     = (parity == SYS_UART_PARITY_EVEN) ? UART_PARITY_EVEN :
                    (parity == SYS_UART_PARITY_ODD) ? UART_PARITY_ODD : UART_PARITY_DISABLE,
      .stop_bits  = (stop_bits == SYS_UART_STOP_BITS_ONE) ? UART_STOP_BITS_1 : UART_STOP_BITS_2,
      .data_bits  = (data_bits == SYS_UART_DATA_BITS_FIVE) ? UART_DATA_5_BITS :
                    (data_bits == SYS_UART_DATA_BITS_SIX) ? UART_DATA_6_BITS :
                    (data_bits == SYS_UART_DATA_BITS_SEVEN) ? UART_DATA_7_BITS : UART_DATA_8_BITS,
      .source_clk = UART_SCLK_APB,
      .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
   };
   uint32_t rx_buffer_len = (config->rx_buffer_len > 0) ? config->rx_buffer_len : SYS_UART_DEFAULT_RX_BUFFER;
   uint8_t idle_symbols = (config->rx_idle_symbols > 0) ? config->rx_idle_symbols : SYS_UART_DEFAULT_RX_IDLE_SYMBOLS;

   l = (esp_uart_t *) malloc(sizeof *l);
   if(NULL == l)
   {
      SERR("[%s] Failed to allocate UART context\n", __func__);
      return NULL;
   }
   memset(l, 0, sizeof *l);
   l->port = config->port;

   if(uart_driver_install(config->port, rx_buffer_len, 0, SYS_UART_EVENT_QUEUE_DEPTH, &l->events, 0) != ESP_OK
   || uart_param_config(config->port, &uart_config) != ESP_OK
   || uart_set_pin(config->port, config->tx_pin, config->rx_pin, -1, -1) != ESP_OK
   || uart_set_rx_timeout(config->port, idle_symbols) != ESP_OK)
   {
      SERR("[%s] UART %u configuration failed\n", __func__, config->port);
      uart_driver_delete(config->port);
      free(l);  /* was leaked */
      return NULL;
   }

   xTaskCreate(uart_ll_rx_task, "uart_rx",
               (config->task_stack > 0) ? config->task_stack : SYS_UART_DEFAULT_TASK_STACK,
               (void *) l,
               (config->task_priority > 0) ? config->task_priority : SYS_UART_DEFAULT_TASK_PRIORITY,
               NULL);

   return l;
}

bool SYS_WEAK uart_ll_deinit(uart_lowlevel_context ctx)
{
   if(NULL == ctx)
      return true;
   free(ctx);
   return true;
}

bool SYS_WEAK uart_ll_set_rx_handler(uart_lowlevel_context ctx, uart_ll_rx_handler_fn handler, void *cookie)
{
   esp_uart_t *l = (esp_uart_t *) ctx;
   if(NULL == l)
      return false;
   l->rx_handler = handler;
   l->rx_cookie = cookie;
   return true;
}

bool SYS_WEAK uart_ll_send(uart_lowlevel_context ctx, uint8_t *data, uint32_t length)
{
   esp_uart_t *l = (esp_uart_t *) ctx;
   if(NULL == l)
      return false;
   /* uart_write_bytes returns the number of bytes queued, not an esp_err_t, so
      comparing it against ESP_OK reported every non-empty write as a failure. */
   return (uart_write_bytes(l->port, (const char *) data, length) == (int) length);
}

/* ----------------------------------------------------------------------------------------------
 * mutex
 */

typedef struct
{
   SemaphoreHandle_t mutex;
} esp_mutex_t;

mutex_lowlevel SYS_WEAK sys_mutex_init(void)
{
   esp_mutex_t *ctx = malloc(sizeof *ctx);
   if(NULL == ctx)
      return NULL;
   ctx->mutex = xSemaphoreCreateMutex();
   if(NULL == ctx->mutex)
   {
      free(ctx);
      return NULL;
   }
   return ctx;
}

bool SYS_WEAK sys_mutex_deinit(mutex_lowlevel mutex)
{
   esp_mutex_t *ctx = (esp_mutex_t *) mutex;
   if(NULL == ctx)
      return true;
   free(ctx);
   return true;
}

bool SYS_WEAK sys_mutex_lock(mutex_lowlevel mutex)
{
   esp_mutex_t *ctx = (esp_mutex_t *) mutex;
   xSemaphoreTake(ctx->mutex, portMAX_DELAY);
   return true;
}

bool SYS_WEAK sys_mutex_unlock(mutex_lowlevel mutex)
{
   esp_mutex_t *ctx = (esp_mutex_t *) mutex;
   xSemaphoreGive(ctx->mutex);
   return true;
}

/* ----------------------------------------------------------------------------------------------
 * time
 */

uint64_t SYS_WEAK sys_microsecond_tick(void)
{
   return esp_timer_get_time(); /* microseconds since boot */
}
