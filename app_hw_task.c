/**
 * @file
 * See header file for description!
 *
 * @copyright 2020 Silicon Laboratories Inc.
 */

#include <zaf_event_distributor_ncp.h>
#include "app_events.h"
#include <SizeOf.h>
#include <zpal_power_manager.h>

// #define DEBUGPRINT
#include <DebugPrint.h>
#include <app_hw_task.h>

/****************************************************************
 * CONFIGURATIONS OF THIS MODULE
 ***************************************************************/

/****************************************************************
 * DEFINITIONS
 ***************************************************************/

/****************************************************************
 * TYPEDEF and CONSTANTS
 ***************************************************************/

/****************************************************************
 * MACROS
 ***************************************************************/

#define USER_TASK_WAKEUP_PERIOD 1000

/****************************************************************
 * FORWARD DECLARATIONS (none preferred)
 ***************************************************************/

/****************************************************************
 * STATIC VARIABLES
 ***************************************************************/
static zpal_pm_handle_t task_power_lock;
/****************************************************************
 * EXTERNAL VARIABLES (none preferred)
 ***************************************************************/

/****************************************************************
 * STATIC FUNCTIONS
 ***************************************************************/

/**
 * This function does the main operation of this task, which is defined by the User.
 */
NO_RETURN static void executeThread(void)
{
  for (;;)
  {
    zpal_pm_stay_awake(task_power_lock, 0);

    // TODO: Read data
    // zaf_event_distributor_enqueue_proprietary_app_event(EVENT_APP_USERTASK_GYRO_MEASUREMENT);

    zpal_pm_cancel(task_power_lock);
    vTaskDelay(pdMS_TO_TICKS(USER_TASK_WAKEUP_PERIOD));
  }
}

/****************************************************************
 * API FUNCTIONS
 ***************************************************************/

/****************************************************************
 * THREAD FUNCTION
 ***************************************************************/

/**
 * A pointer to this function is passed to ZW_UserTask_CreateTask().
 *
 * ATTENTION: This task context shall never call a ZAF API function!
 *
 * The pointer passed to this function is user defined (void-pointer),
 * but here casted to SApplicationHandles-pointer.
 */
NO_RETURN void
NC_UserTask_Hardware(__attribute__((unused)) void *pUserTaskParam)
{
  DPRINT("\r\nHardware UserTask started!");

  task_power_lock = zpal_pm_register(ZPAL_PM_TYPE_DEEP_SLEEP);

  // Generate event that says the Data acquisition UserTask has started!
  if (zaf_event_distributor_enqueue_proprietary_app_event(EVENT_APP_USERTASK_READY))
  {
    DPRINT("\r\nNC_UserTask: Ready event is sent to main app!\r\n");
  }

  executeThread();
}
