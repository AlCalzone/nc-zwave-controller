#ifndef NC_USER_TASK_EVENTS_H_
#define NC_USER_TASK_EVENTS_H_

#include <ev_man.h>

/**
 * Defines events for the application.
 *
 * These events are not referred to anywhere else than in the application. Hence, they can be
 * altered to suit the application flow.
 *
 * The events are located in a separate file to make it possible to include them in other
 * application files. An example could be a peripheral driver that enqueues an event when something
 * specific happens, e.g. on motion detection.
 */
typedef enum EVENT_APP_NC
{
  EVENT_EMPTY = DEFINE_EVENT_APP_NBR,
  /*
   * User-Task related events towards the Main App!
   */
  EVENT_APP_USERTASK_READY,
  EVENT_APP_USERTASK_GYRO_MEASUREMENT,
}
EVENT_APP;

#endif /* NC_USER_TASK_EVENTS_H_ */
