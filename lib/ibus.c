/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \file ibus.c
 *  \brief Flysky IBUS protocol parser implementation
 */
#include "ibus/ibus.h"
#include "helpers.h"
#include "sys/sys.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define IBUS_BAUD                115200
#define IBUS_DATA_BITS           8
#define IBUS_PARITY              SYS_UART_PARITY_NONE
#define IBUS_STOP_BITS           SYS_UART_STOP_BITS_ONE
#define IBUS_MAX_BUFFER_LENGTH   0x20
#define IBUS_PROTOCOL_OVERHEAD   3     // Don't include 1-byte length or 2-byte checksum

/* A gap this long between received bytes ends a message. At 115200 baud a byte
   takes 87 us and the gap between servo messages is about 4 ms, so anything
   between the two delimits them; 1 ms sits in the middle of that. Without a
   non-zero value here the parser has no gap to synchronise on and can only
   recover from a bad lock by failing a checksum. */
#define IBUS_DEFAULT_RX_TIMEOUT  1000

typedef enum
{
   IBUS_CMD_SERVO       = 4,
   IBUS_CMD_DISCIVER    = 8,
} ibus_cmd_e;

typedef enum
{
   IBSTATE_GET_LENGTH = 0,
   IBSTATE_GET_DATA,
   IBSTATE_GET_CHECKSUM_LOW,
   IBSTATE_GET_CHECKSUM_HIGH
} ibus_state_e;

typedef struct ibus_state_s
{
   ibus_state_e state;
   uint8_t ptr;
   uint8_t len;
   uint16_t checksum;
   uint16_t checksum_expected;
   uint8_t buffer[IBUS_MAX_BUFFER_LENGTH];
   uint64_t timestamp_max; /* maximum possible timestamp value before wrapping */
   uint64_t rx_timeout;
   uint64_t last_rx; /* timestamp of most recent rx character */
   ibus_channel_t channels[IBUS_CHANNEL_COUNT];

   ibus_statistics_t stats;
   ibus_handle_channel_update_fn channel_handler;
   void *channel_handler_cookie;
   void *lowlevel;
} ibus_state_t;

/* ----------------------------------------------------------------------------------------
 * Private Functions
 */

static void ibus_HandleBuffer(ibus_state_t *state)
{
   //uint8_t address = state->buffer[0] & 0x0f;
   uint8_t command = (state->buffer[0] & 0xf0) >> 4;
   switch(command)
   {
      case IBUS_CMD_SERVO:
         if(state->len != 29)
            break;
         for(int i = 0; i < IBUS_CHANNEL_COUNT; ++i)
         {
            ibus_channel_t *channel = &state->channels[i];
            uint16_t value = state->buffer[1 + (i*2)]
                           | ((uint16_t)(state->buffer[2 + (i*2)]) << 8);
            channel->value = value;
            if(channel->min > value)
               channel->min = value;
            if(channel->max < value)
               channel->max = value;
            ++(channel->update_count);
         }
         if(NULL != state->channel_handler)
            state->channel_handler(state->channels, state->channel_handler_cookie);
         break;
   }
}

static void ibus_ll_HandleRxBuffer(uint8_t *data, uint32_t length, void *cookie)
{
   ibus_state_t *state = (ibus_state_t *) cookie;
   uint64_t time = sys_microsecond_tick();
   for(int c = 0; c < length; ++c)
      ibus_handle_byte(state, data[c], time);
}

/* ----------------------------------------------------------------------------------------
 * Exported Functions
 */

ibus_context_t ibus_init(uart_lowlevel_config *config)
{
   ibus_state_t *state = (ibus_state_t *) malloc(sizeof(*state));
   if(NULL == state)
      return NULL;

   memset(state, 0, sizeof(*state));
   state->state = IBSTATE_GET_LENGTH;
   ibus_reset_statistics(state);
   for(int idx = 0; idx < IBUS_CHANNEL_COUNT; ++idx)
      ibus_reset_channel(state, idx);

   /* These were documented as configuration but never read out of it, so they
      were always zero: every byte looked like it followed an over-long gap,
      which reset the parser constantly and made timeout_count meaningless. */
   if(NULL != config)
   {
      state->rx_timeout = config->rx_timeout;
      state->timestamp_max = config->timestamp_max;
   }
   if(0 == state->rx_timeout)
      state->rx_timeout = IBUS_DEFAULT_RX_TIMEOUT;
   if(0 == state->timestamp_max)
      state->timestamp_max = UINT64_MAX;

   #if defined(IBUS_LOWLEVEL_NONE)
   state->lowlevel = NULL;
   #else
   state->lowlevel = uart_ll_init(IBUS_BAUD, IBUS_DATA_BITS, IBUS_STOP_BITS, IBUS_PARITY, config);
   if(NULL == state->lowlevel)
   {
      SERR("[%s] Low-level initialization failed\n", __func__);
      free(state);
      return NULL;
   }
   if(!uart_ll_set_rx_handler(state->lowlevel, ibus_ll_HandleRxBuffer, state))
   {
      SERR("[%s] Failed to configure Rx data handler\n", __func__);
   }
   #endif

   return (ibus_context_t) state;
}

bool ibus_deinit(ibus_context_t context)
{
   ibus_state_t *state = (ibus_state_t *) context;
   if(NULL == state)
      return true;

   uart_ll_deinit(state->lowlevel);
   free(state);
   return true;
}

bool ibus_set_channel_handler(ibus_context_t context, ibus_handle_channel_update_fn handler, void *cookie)
{
   ibus_state_t *state = (ibus_state_t *) context;
   if(NULL == state)
      return false;
   state->channel_handler = handler;
   state->channel_handler_cookie = cookie;
   return true;
}

bool ibus_get_statistics(ibus_context_t context, ibus_statistics_t *stats)
{
   ibus_state_t *state = (ibus_state_t *) context;
   if(NULL == state || NULL == stats)
      return false;
   memcpy(stats, &state->stats, sizeof *stats);
   return true;
}

bool ibus_reset_statistics(ibus_context_t context)
{
   ibus_state_t *state = (ibus_state_t *) context;
   if(NULL == state)
      return false;

   memset(&state->stats, 0, sizeof state->stats);
   state->stats.min_delay = UINT_MAX;
   return true;
}

bool ibus_reset_channel(ibus_context_t context, uint8_t channel)
{
   ibus_state_t *state = (ibus_state_t *) context;
   if(NULL == state || channel >= IBUS_CHANNEL_COUNT)
      return false;
   ibus_channel_t *c = &state->channels[channel];
   memset(c, 0, sizeof *c);
   c->min = USHRT_MAX;
   return true;
}

void ibus_handle_byte(ibus_context_t context, uint8_t val, uint64_t timestamp)
{
   ibus_state_t *state = (ibus_state_t *) context;
   uint64_t delta_rx;

   ++(state->stats.rx_byte_count);

   if(timestamp > 0)
   {
      if(timestamp < state->last_rx)
         delta_rx = state->timestamp_max - state->last_rx + timestamp;
      else
         delta_rx = timestamp - state->last_rx;
      if(delta_rx > state->rx_timeout)
      {
         ++(state->stats.timeout_count);
         state->state = IBSTATE_GET_LENGTH;
      }
      if(state->stats.max_delay < delta_rx)
         state->stats.max_delay = delta_rx;
      if(state->stats.min_delay > delta_rx)
         state->stats.min_delay = delta_rx;
   }
   state->last_rx = timestamp;

   switch(state->state)
   {
      case IBSTATE_GET_LENGTH:
         if(val <= IBUS_MAX_BUFFER_LENGTH && val > IBUS_PROTOCOL_OVERHEAD)
         {
            state->ptr = 0;
            state->len = val - IBUS_PROTOCOL_OVERHEAD;
            state->checksum = 0xFFFF - val;
            state->state = IBSTATE_GET_DATA;
        }
        break;

      case IBSTATE_GET_DATA:
         state->buffer[state->ptr] = val;
         state->ptr++;
         state->checksum -= val;
         if(state->ptr >= state->len)
            state->state = IBSTATE_GET_CHECKSUM_LOW;
         break;

      case IBSTATE_GET_CHECKSUM_LOW:
         state->checksum_expected = val;
         state->state = IBSTATE_GET_CHECKSUM_HIGH;
         break;

      case IBSTATE_GET_CHECKSUM_HIGH:
         state->checksum_expected |= ((uint16_t) val) << 8;
         if(state->checksum == state->checksum_expected)
         {
            ibus_HandleBuffer(state);
            ++(state->stats.rx_message_count);
         }
         else
            ++(state->stats.rx_message_checksum_fail_count);
         state->state = IBSTATE_GET_LENGTH;
         break;

      default:
         break;
   }
}