/**
 * @file cmd_handlers_invoker.c
 * @copyright 2022 Silicon Laboratories Inc.
 */

#include "cmd_handlers.h"
#include <assert.h>

/**
 * This is the first of the registered handlers
 */
extern const cmd_handler_map_t __start_zw_cmd_handlers;
#define cmd_handlers_start __start_zw_cmd_handlers
/**
 * This marks the end of the handlers. The element
 * after the last element. This means that this element
 * is not valid.
 */
extern const cmd_handler_map_t __stop_zw_cmd_handlers;
#define cmd_handlers_stop __stop_zw_cmd_handlers


bool invoke_cmd_handler(const comm_interface_frame_ptr frame)
{
  cmd_handler_map_t const * iter = &cmd_handlers_start;
  for ( ; iter < &cmd_handlers_stop; ++iter)
  {
    if (iter->cmd == frame->cmd) {
      iter->pHandler(frame);
      return true;
    }
  }

  return false;
}

void cmd_foreach(cmd_foreach_callback_t callback, uint8_t* supported_cmds_mask)
{
  assert(callback != NULL);
  cmd_handler_map_t const * iter = &cmd_handlers_start;
  for ( ; iter < &cmd_handlers_stop; ++iter)
  {
    if (true == callback(iter, supported_cmds_mask)) {
      break;
    }
  }
}
