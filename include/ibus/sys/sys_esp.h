/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief esp-idf lowlevel portability interface
 *  \version 2.0
 */
#ifndef _SYS_ESP_IDF_H
#define _SYS_ESP_IDF_H

#include "hal/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"

typedef struct i2c_lowlevel_s
{
   /* If bus == NULL, port, pin_sda, and pin_scl will be used to
      initialize the I2C bus, otherwise it's assumed that a previous
      call was made to i2c_new_master_bus, and the resulting handle
      is assigned to this 'bus' variable.  */
   i2c_master_bus_handle_t *bus;

   /* If bus != NULL, the following variables will be used to
      initialize the I2C bus. */
   i2c_port_t port;
   int pin_sda;
   int pin_scl;
} i2c_lowlevel_config;

typedef struct uart_lowlevel_s
{
   uart_port_t port;  /* e.g. UART_NUM_2 */
   int tx_pin;        /* UART_PIN_NO_CHANGE for a receive-only link */
   int rx_pin;

   /* All of the following may be left zero, for the defaults below. */

   /* How much idle line ends a received burst, in symbol periods. This is the
      one setting that decides whether the parser can see message boundaries at
      all.

      The parser delimits messages by the gap between received bytes, but the
      timestamp it is given is taken once per callback -- so the only gaps it
      can see are the gaps between callbacks. Handing it fixed-size reads makes
      those boundaries fall wherever the UART driver split the stream, which is
      not where the messages are, and a split that lands mid-message stays
      mid-message for as long as the link is up. Delivering on the line going
      idle instead makes each callback one message, which is what the parser
      assumes.

      Set it above one message's inter-byte spacing and below the gap between
      messages. Ten symbols is about 870 us at 115200 baud, which sits between
      a byte time of 87 us and the ~4 ms between IBUS servo messages. */
   uint8_t rx_idle_symbols;   /* default SYS_UART_DEFAULT_RX_IDLE_SYMBOLS */

   uint32_t rx_buffer_len;    /* default SYS_UART_DEFAULT_RX_BUFFER */
   uint32_t task_stack;       /* default SYS_UART_DEFAULT_TASK_STACK */
   uint32_t task_priority;    /* default SYS_UART_DEFAULT_TASK_PRIORITY */

   /* How long a gap between received bytes ends a message, in the units of
      sys_microsecond_tick() -- microseconds. Mirrors the field of the same
      name on platforms with no low-level implementation, so a caller sets it
      the same way everywhere. */
   uint64_t rx_timeout;
   uint64_t timestamp_max;    /* tick value at which the clock wraps; 0 = 64-bit */
} uart_lowlevel_config;

#define SYS_UART_DEFAULT_RX_IDLE_SYMBOLS 10
#define SYS_UART_DEFAULT_RX_BUFFER       256
#define SYS_UART_DEFAULT_TASK_STACK      4096
#define SYS_UART_DEFAULT_TASK_PRIORITY   5

#endif /* _SYS_ESP_IDF_H */
