/**
 * @file
 * @brief ZAF Event distributor source file
 * @copyright 2019 Silicon Laboratories Inc.
 */
#include <AppTimer.h>
#include <EventDistributor.h>
#include <SyncEvent.h>
#include <SizeOf.h>
#include <ZW_typedefs.h>
#include <ZAF_Common_helper.h>
#include <ZAF_Common_interface.h>
#include <zaf_event_distributor_ncp.h>
#include <ZW_TransportSecProtocol.h>

//#define DEBUGPRINT
#include "DebugPrint.h"

// Event distributor object
static SEventDistributor g_EventDistributor = { 0 };

static void EventHandlerZwRx(void);
static void EventHandlerZwCommandStatus(void);
static void EventHandlerProprietary(void);

// Event distributor event handler table
static const EventDistributorEventHandler g_aEventHandlerTable[7] =
{
  EventHandlerZwRx,                         // EAPPLICATIONEVENT_ZWRX = 0
  EventHandlerZwCommandStatus,              // EAPPLICATIONEVENT_ZWCOMMANDSTATUS = 1
  zaf_event_distributor_app_state_change,   // EAPPLICATIONEVENT_STATECHANGE  = 2
  zaf_event_distributor_app_serial_data_rx, // EAPPLICATIONEVENT_SERIALDATARX = 3
  zaf_event_distributor_app_serial_timeout, // EAPPLICATIONEVENT_SERIALTIMEOUT = 4
  AppTimerNotificationHandler,              // EAPPLICATIONEVENT_TIMER = 5
  EventHandlerProprietary                   // EAPPLICATIONEVENT_PROPRIETARY = 6
};

/**
 * Custom event queues for proprietary events.
 */
#define NC_EVENT_QUEUE_SIZE 5
static SQueueNotifying m_NCEventNotifyingQueue = { 0 };
static StaticQueue_t m_NCEventQueueObject = { 0 };
static event_nc_t m_NCEventQueueStorage[NC_EVENT_QUEUE_SIZE] = { { 0 } };
static QueueHandle_t m_NCEventQueue = { 0 };

static void EventHandlerZwRx(void)
{
  SApplicationHandles* pAppHandles;
  SZwaveReceivePackage RxPackage = { 0 };

  pAppHandles = ZAF_getAppHandle();

  // Handle incoming replies
  while (xQueueReceive(pAppHandles->ZwRxQueue, (uint8_t *)(&RxPackage), 0) == pdTRUE) {
    DPRINTF("Incoming Rx %x \r\n", RxPackage.eReceiveType);

    switch (RxPackage.eReceiveType) {
#ifdef ZW_CONTROLLER_BRIDGE
      case EZWAVERECEIVETYPE_MULTI:
#endif // #ifdef ZW_CONTROLLER_BRIDGE
#ifdef ZW_SLAVE
      case EZWAVERECEIVETYPE_SECURITY_EVENT:
#endif
      case EZWAVERECEIVETYPE_STAY_AWAKE:
      case EZWAVERECEIVETYPE_NODE_UPDATE:
      case EZWAVERECEIVETYPE_SINGLE:
        break;

      default:
        DPRINTF("Invalid Receive Type: %d", RxPackage.eReceiveType);
        break;
    }

    zaf_event_distributor_app_zw_rx(&RxPackage);
  }
}

/**
 * @brief Triggered when protocol puts a message on the ZwCommandStatusQueue.
 */
static void EventHandlerZwCommandStatus(void)
{
  SApplicationHandles* pAppHandles;
  SZwaveCommandStatusPackage Status = { 0 };

  pAppHandles = ZAF_getAppHandle();

  // Handle incoming replies
  while (xQueueReceive(pAppHandles->ZwCommandStatusQueue, (uint8_t*)(&Status), 0) == pdTRUE) {
    {
      DPRINTF("Incoming Status msg %x\r\n", Status.eStatusType);

      switch (Status.eStatusType) {
        case EZWAVECOMMANDSTATUS_TX:
        {
          SZWaveTransmitStatus *pTxStatus = &Status.Content.TxStatus;
          if (!pTxStatus->bIsTxFrameLegal) {
            DPRINT("Auch - not sure what to do\r\n");
          } else {
            DPRINT("Tx Status received\r\n");
            if (pTxStatus->Handle) {
              ZW_TX_Callback_t pCallback = (ZW_TX_Callback_t)pTxStatus->Handle;
              pCallback(pTxStatus->TxStatus, &pTxStatus->ExtendedTxStatus);
            }
          }

          break;
        }

#ifdef ZW_CONTROLLER
        case EZWAVECOMMANDSTATUS_REPLACE_FAILED_NODE_ID:
        case EZWAVECOMMANDSTATUS_REMOVE_FAILED_NODE_ID:
        case EZWAVECOMMANDSTATUS_NETWORK_MANAGEMENT:
#if SUPPORT_ZW_SET_SLAVE_LEARN_MODE
        case EZWAVECOMMANDSTATUS_SET_SLAVE_LEARN_MODE:
#endif
#endif
        case EZWAVECOMMANDSTATUS_LEARN_MODE_STATUS:
        case EZWAVECOMMANDSTATUS_SET_DEFAULT:
          break;

        case EZWAVECOMMANDSTATUS_INVALID_TX_REQUEST:
          DPRINTF("Invalid TX Request to protocol - %d\r\n", Status.Content.InvalidTxRequestStatus.InvalidTxRequest);
          break;

        case EZWAVECOMMANDSTATUS_INVALID_COMMAND:
          DPRINTF("Invalid command to protocol - %d\r\n", Status.Content.InvalidCommandStatus.InvalidCommand);
          break;

        default:
          DPRINTF("Unknown / Unexpected  CMD - %d\r\n", Status.eStatusType);
          break;
      }
      zaf_event_distributor_app_zw_command_status(&Status);
    }
  }
}

/*
 * Initializes proprietary event queues
 */
static void
EventQueueInit(void)
{
  // Initialize Queue Notifier for events in the application.
  m_NCEventQueue = xQueueCreateStatic(
    sizeof_array(m_NCEventQueueStorage),
    sizeof(m_NCEventQueueStorage[0]),
    (uint8_t*)m_NCEventQueueStorage,
    &m_NCEventQueueObject
    );

  /*
   * Registers events with associated data, and notifies
   * the specific task about a pending job
   */
  QueueNotifyingInit(
    &m_NCEventNotifyingQueue,
    m_NCEventQueue,
    xTaskGetCurrentTaskHandle(),
    6 /* EAPPLICATIONEVENT_PROPRIETARY */
  );
}

void zaf_event_distributor_init(void)
{
  EventQueueInit();

  EventDistributorConfig(
    &g_EventDistributor,
    sizeof_array(g_aEventHandlerTable),
    g_aEventHandlerTable,
    NULL);
}

const SEventDistributor *zaf_event_distributor_get(void)
{
  return &g_EventDistributor;
}

ZW_WEAK void
zaf_event_distributor_app_zw_rx(__attribute__((unused)) SZwaveReceivePackage *RxPackage)
{
}

ZW_WEAK void
zaf_event_distributor_app_zw_command_status(__attribute__((unused)) SZwaveCommandStatusPackage *Status)
{
}

bool zaf_event_distributor_enqueue_proprietary_app_event(const uint8_t event, nc_event_payload_t *payload)
{
  EQueueNotifyingStatus Status = EQUEUENOTIFYING_STATUS_TIMEOUT;
  const event_nc_t event_nc = {
    .event = event,
    .payload = payload,
  };
  bool returnValue = false;

  Status = QueueNotifyingSendToBack(&m_NCEventNotifyingQueue, (const uint8_t*) &event_nc, 0);

  switch (Status) {
    case EQUEUENOTIFYING_STATUS_SUCCESS:
      returnValue = true;
      break;
    case EQUEUENOTIFYING_STATUS_WRONG_PARAMETER:
      DPRINT("Failed to queue event because of wrong input parameter\n");
      returnValue = false;
      break;
    case EQUEUENOTIFYING_STATUS_TIMEOUT:
      DPRINT("Failed to queue event because of timeout\n");
      returnValue = false;
      break;
    default:
      returnValue = false;
      break;
  }

  return returnValue;
}

bool zaf_event_distributor_enqueue_proprietary_app_event_from_isr(const uint8_t event, nc_event_payload_t *payload)
{
  EQueueNotifyingStatus Status;
  const event_nc_t event_nc = {
    .event = event,
    .payload = payload,
  };
  bool returnValue = false;

  Status = QueueNotifyingSendToBackFromISR(&m_NCEventNotifyingQueue, (const uint8_t*) &event_nc);

  switch (Status) {
    case EQUEUENOTIFYING_STATUS_SUCCESS:
      returnValue = true;
      break;
    case EQUEUENOTIFYING_STATUS_WRONG_PARAMETER:
      DPRINT("Failed to queue event because of wrong input parameter\n");
      returnValue = false;
      break;
    case EQUEUENOTIFYING_STATUS_TIMEOUT:
      DPRINT("Failed to queue event because of timeout\n");
      returnValue = false;
      break;
    default:
      returnValue = false;
      break;
  }

  return returnValue;
}

static void EventHandlerProprietary(void)
{
  event_nc_t event_nc = { 0 };

  while (xQueueReceive(m_NCEventQueue, (uint8_t*)(&event_nc), 0) == pdTRUE) {
    DPRINTF("Proprietary Event: %d\n", event_nc.event);
    zaf_event_distributor_app_proprietary(&event_nc);
  }
}
