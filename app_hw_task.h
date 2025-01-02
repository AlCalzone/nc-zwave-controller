/**
 * @file
 *
 * A break-out example of data acquisition operations for a sensor device to a
 * different UserTask than the Main Application Task.
 *
 * All sensor signals are monitored by this task and passed on to the
 * Main Application Task (MAT) via FreeRTOS IPCs mechanisms.
 *
 * ATTENTION: This module may not call any ZAF API function, except for the
 * zpal_pm_stay_awake() function!
 *
 * @copyright 2020 Silicon Laboratories Inc.
 */

#ifndef _USER_TASK_DATA_ACQUISITION_H__
#define _USER_TASK_DATA_ACQUISITION_H__

/****************************************************************************
 * INCLUDE FILES
 ***************************************************************************/
#include <ZW_UserTask.h>
#include <ev_man.h>

/****************************************************************************
 * CONFIGURATIONS*
 ***************************************************************************/

/****************************************************************************
 * DEFINITIONS, TYPEDEF and CONSTANTS
 ***************************************************************************/

/****************************************************************************
 * MACROS*
 ***************************************************************************/

/****************************************************************************
 * EXTERNAL VARIABLES*
 ***************************************************************************/

/****************************************************************************
 * ENUMERATIONS
 ***************************************************************************/

/****************************************************************************
 * API FUNCTIONS
 ***************************************************************************/

/****************************************************************************
 * THREAD FUNCTION*
 ***************************************************************************/
/**
 * @brief This is the task function. It must never return!
 */
NO_RETURN void NC_UserTask_Hardware(void* pAppHandles);

#endif // _GYRO_USER_TASK_H__
