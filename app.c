/**
 * @file
 * Serial API implementation for Enhanced Z-Wave module
 *
 * @copyright 2019 Silicon Laboratories Inc.
 */

#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include "SyncEvent.h"
#ifdef ZW_CONTROLLER
#include "ZW_controller_api.h"
#endif /* ZW_CONTROLLER */
#include "AppTimer.h"
#include "ZW_system_startup_api.h"
#include "zpal_retention_register.h"
/* Include app header file - containing version and */
/* SerialAPI functionality support definitions */
#ifdef ZW_SECURITY_PROTOCOL
#include "ZW_security_api.h"
#include "ZW_TransportSecProtocol.h"
#endif
#include "DebugPrintConfig.h"
// SerialAPI uses SWO for debug output.
// For example SWO Terminal in  Studio commander can be used to get the output.
//#define DEBUGPRINT
#include "DebugPrint.h"
#include "app_node_info.h"
#include "serialapi_file.h"
#include "cmd_handlers.h"
#include "cmds_management.h"
#include "ZAF_Common_interface.h"
#include "utils.h"
#include "app_hw.h"
#include "SerialAPI_hw.h"
#include "zaf_event_distributor_ncp.h"
#include "zpal_misc.h"
#include "zpal_watchdog.h"
#include "zaf_protocol_config.h"
#ifdef DEBUGPRINT
#include "ZAF_PrintAppInfo.h"
#endif
#include "ZAF_AppName.h"
#include <ZAF_nvm_app.h>

#include <assert.h>

#if (!defined(SL_CATALOG_SILICON_LABS_ZWAVE_APPLICATION_PRESENT) && !defined(UNIT_TEST))
#include "app_hw.h"
#endif

#include "drivers/ws2812.h"
#include "drivers/qma6100p.h"
#include "cmds_proprietary.h"

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))


/********************************
 * Data Acquisition Task
 *******************************/
#include "app_hw_task.h"
#include "app_events.h"
#include "sl_button.h"

#define HW_TASK_STACK_SIZE           1000  // [bytes]
static TaskHandle_t m_xTaskHandleBackgroundHw   = NULL;
// Task and stack buffer allocation for the default/main application task!
static StaticTask_t BackgroundHwTaskBuffer;
static uint8_t BackgroundHwStackBuffer[HW_TASK_STACK_SIZE];

// Gyro measurements
bool bRequestGyroMeasurement = false;
gyro_reading_t last_gyro_reading = {0};
int stable_gyro_readings = 0;

bool bAwaitingConnection = false;
// Whether incorrect tilt was detected and should be indicated
bool bTiltDetected = false;
bool bEnableTiltDetection = true;

// Define thresholds and deadzone for tilt detection to compensate for noise/drift
#define TILT_THRESHOLD_LOWER  12.0f
#define TILT_THRESHOLD_UPPER  16.0f
#define GYRO_STABLE_THRESHOLD 3

// Default LED effect when no other effect is set
LedEffect_t ledEffectDefault = {0};
// LED state set by the user
LedEffect_t ledEffectUser = {0};
// Separate high priority LED state for system indication
LedEffect_t ledEffectSystem = {0};
// LED state to indicate incorrect tilt
LedEffect_t ledEffectTilt = {0};

static void ApplicationInitSW(void);
static void ApplicationTask(SApplicationHandles *pAppHandles);



/* Basic level definitions */
#define BASIC_ON 0xFF
#define BASIC_OFF 0x00

#define TX_POWER_LR_20_DBM    200
#define TX_POWER_LR_14_DBM    140

#ifdef ZW_SECURITY_PROTOCOL
#define REQUESTED_SECURITY_KEYS   ( SECURITY_KEY_S0_BIT | SECURITY_KEY_S2_UNAUTHENTICATED_BIT | SECURITY_KEY_S2_AUTHENTICATED_BIT | SECURITY_KEY_S2_ACCESS_BIT)
#else
#define REQUESTED_SECURITY_KEYS   0
#endif  /* ZW_SECURITY_PROTOCOL */

/* Accept all incoming command classes, regardless of NIF contents. */
#define ACCEPT_ALL_CMD_CLASSES

/**
 *
 */
typedef struct _S_TRANSPORT_REQUESTED_SECURITY_SETTINGS_
{
  uint8_t requestedSecurityKeysBits;
} S_TRANSPORT_REQUESTED_SECURITY_SETTINGS;

static TaskHandle_t g_AppTaskHandle;

extern SSyncEvent SetDefaultCB;
extern SSyncEventArg1 LearnModeStatusCb;

/* State vars for ApplicationPoll */
static uint8_t state = 0xff;
static uint8_t retry = 0;

static uint8_t lastRetVal = 0;      /* Used to store retVal for retransmissions */
uint8_t compl_workbuf[BUF_SIZE_TX]; /* Used for frames send to remote side. */

/* Queue for frames transmitted to PC - callback, ApplicationCommandHandler, ApplicationControllerUpdate... */
#if !defined(MAX_CALLBACK_QUEUE)
#define MAX_CALLBACK_QUEUE  8
#endif /* !defined(MAX_CALLBACK_QUEUE) */

#if !defined(MAX_UNSOLICITED_QUEUE)
#define MAX_UNSOLICITED_QUEUE 8
#endif /* !defined(MAX_UNSOLICITED_QUEUE) */

typedef struct _callback_element_
{
  uint8_t wCmd;
  uint8_t wLen;
  uint8_t wBuf[BUF_SIZE_TX];
} CALLBACK_ELEMENT;

typedef struct _request_queue_
{
  uint8_t requestOut;
  uint8_t requestIn;
  uint8_t requestCnt;
  CALLBACK_ELEMENT requestQueue[MAX_CALLBACK_QUEUE];
} REQUEST_QUEUE;

REQUEST_QUEUE callbackQueue = {0};

typedef struct _request_unsolicited_queue_
{
  uint8_t requestOut;
  uint8_t requestIn;
  uint8_t requestCnt;
  CALLBACK_ELEMENT requestQueue[MAX_UNSOLICITED_QUEUE];
} REQUEST_UNSOLICITED_QUEUE;

REQUEST_UNSOLICITED_QUEUE commandQueue = {0};

eSerialAPISetupNodeIdBaseType nodeIdBaseType = SERIAL_API_SETUP_NODEID_BASE_TYPE_DEFAULT;

#if SUPPORT_ZW_WATCHDOG_START | SUPPORT_ZW_WATCHDOG_STOP
extern uint8_t bWatchdogStarted;
#endif

/* Last system wakeup reason - is set in ApplicationInit */
zpal_reset_reason_t g_eApplResetReason;

zpal_pm_handle_t radio_power_lock;
zpal_pm_handle_t io_power_lock;
SSwTimer mWakeupTimer;
bool bTxStatusReportEnabled;


static void ApplicationInitSW(void);
static void ApplicationTask(SApplicationHandles *pAppHandles);

#if (defined(SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION) && SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION )
static bool request_protocol_cc_encryption(SZwaveReceivePackage *pRPCCEPackage);
#endif

#ifdef ZW_CONTROLLER_BRIDGE
static void ApplicationCommandHandler_Bridge(SReceiveMulti *pReciveMulti);
#else
void ApplicationCommandHandler(void *pSubscriberContext, SZwaveReceivePackage* pRxPackage);
#endif

void ApplicationNodeUpdate(uint8_t bStatus, uint16_t nodeID, uint8_t *pCmd, uint8_t bLen);

#if SUPPORT_ZW_REMOVE_FAILED_NODE_ID
extern void ZCB_ComplHandler_ZW_RemoveFailedNodeID(uint8_t bStatus);
#endif

#if SUPPORT_ZW_REPLACE_FAILED_NODE
extern void ZCB_ComplHandler_ZW_ReplaceFailedNode(uint8_t bStatus);
#endif

#if SUPPORT_ZW_SET_SLAVE_LEARN_MODE
extern void ZCB_ComplHandler_ZW_SetSlaveLearnMode(uint8_t bStatus, uint8_t orgID, uint8_t newID);
#endif

#if SUPPORT_ZW_SET_RF_RECEIVE_MODE
extern uint8_t SetRFReceiveMode(uint8_t mode);
#endif

void set_state_and_notify(uint8_t st)
{
  if (state != st)
  {
    xTaskNotify(g_AppTaskHandle,
                1<<EAPPLICATIONEVENT_STATECHANGE,
                eSetBits );
    state = st;
  }
}

void set_state(uint8_t st)
{
  state = st;
}

/*===============================   Request   ================================
**    Queues request (callback) to be transmitted to remote side
**
**--------------------------------------------------------------------------*/
bool /*RET  queue status (false queue full)*/
Request(
    uint8_t cmd,       /*IN   Command                  */
    uint8_t *pData, /*IN   pointer to data          */
    uint8_t len        /*IN   Length of data           */
)
{
  if (callbackQueue.requestCnt < MAX_CALLBACK_QUEUE)
  {
    callbackQueue.requestCnt++;
    callbackQueue.requestQueue[callbackQueue.requestIn].wCmd = cmd;
    if (len > (uint8_t)BUF_SIZE_TX)
    {
      assert((uint8_t)BUF_SIZE_TX >= len);
      len = (uint8_t)BUF_SIZE_TX;
    }
    callbackQueue.requestQueue[callbackQueue.requestIn].wLen = len;
    memcpy(&callbackQueue.requestQueue[callbackQueue.requestIn].wBuf[0], pData, len);
    if (++callbackQueue.requestIn >= MAX_CALLBACK_QUEUE)
    {
      callbackQueue.requestIn = 0;
    }
    xTaskNotify(g_AppTaskHandle,
                1<<EAPPLICATIONEVENT_STATECHANGE,
                eSetBits);

    return true;
  }
  return false;
}

/*=========================   RequestUnsolicited   ===========================
**    Queues request (command) to be transmitted to remote side
**
**--------------------------------------------------------------------------*/
bool /*RET  queue status (false queue full)*/
RequestUnsolicited(
    uint8_t cmd,       /*IN   Command                  */
    uint8_t *pData, /*IN   pointer to data          */
    uint8_t len        /*IN   Length of data           */
)
{
  taskENTER_CRITICAL();
  if (commandQueue.requestCnt < MAX_UNSOLICITED_QUEUE)
  {
    commandQueue.requestCnt++;
    commandQueue.requestQueue[commandQueue.requestIn].wCmd = cmd;
    if (len > (uint8_t)BUF_SIZE_TX)
    {
      assert((uint8_t)BUF_SIZE_TX >= len);
      len = (uint8_t)BUF_SIZE_TX;
    }
    commandQueue.requestQueue[commandQueue.requestIn].wLen = len;
    memcpy(&commandQueue.requestQueue[commandQueue.requestIn].wBuf[0], pData, len);
    if (++commandQueue.requestIn >= MAX_UNSOLICITED_QUEUE)
    {
      commandQueue.requestIn = 0;
    }
    taskEXIT_CRITICAL();
    xTaskNotify(g_AppTaskHandle,
                1<<EAPPLICATIONEVENT_STATECHANGE,
                eSetBits);
    return true;
  }
  taskEXIT_CRITICAL();
  return false;
}

void PurgeCallbackQueue(void)
{
  callbackQueue.requestOut = callbackQueue.requestIn = callbackQueue.requestCnt = 0;
}

void PurgeCommandQueue(void)
{
  taskENTER_CRITICAL();
  commandQueue.requestOut = commandQueue.requestIn = commandQueue.requestCnt = 0;
  taskEXIT_CRITICAL();
}

/*===============================   Respond   ===============================
**    Send immediate respons to remote side
**
**    Side effects: Sets state variable to stateTxSerial (wait for ack)
**
**--------------------------------------------------------------------------*/
void /*RET  Nothing                 */
Respond(
    uint8_t cmd,             /*IN   Command                  */
    uint8_t const *pData, /*IN   pointer to data          */
    uint8_t len              /*IN   Length of data           */
)
{
  /* If there are no data; pData == NULL and len == 0 we must set the data pointer */
  /* to some dummy data. comm_interface_transmit_frame interprets NULL pointer as retransmit indication */
  if (len == 0)
  {
    pData = (uint8_t *)0x7ff; /* Just something is not used anyway */
  }
  comm_interface_transmit_frame(cmd, RESPONSE, pData, len, NULL);

  set_state_and_notify(stateTxSerial); /* We want ACK/NAK...*/
}

void
DoRespond(uint8_t retVal)
{
  /* We need to store retVal for retransmission. */
  lastRetVal = retVal;
  Respond(serial_frame->cmd, &lastRetVal, 1);
}

void
DoRespond_workbuf(
  uint8_t cnt
)
{
  Respond(serial_frame->cmd, compl_workbuf, cnt);
}

void zaf_event_distributor_app_zw_rx(SZwaveReceivePackage *RxPackage)
{
  switch (RxPackage->eReceiveType) {
    case EZWAVERECEIVETYPE_SINGLE:
#ifndef ZW_CONTROLLER_BRIDGE
      ApplicationCommandHandler(NULL, RxPackage);
#endif
      break;

#ifdef ZW_CONTROLLER_BRIDGE
    case EZWAVERECEIVETYPE_MULTI:
      ApplicationCommandHandler_Bridge(&RxPackage->uReceiveParams.RxMulti);
      break;
#endif // #ifdef ZW_CONTROLLER_BRIDGE

    case EZWAVERECEIVETYPE_NODE_UPDATE:
      ApplicationNodeUpdate(
        RxPackage->uReceiveParams.RxNodeUpdate.Status,
        RxPackage->uReceiveParams.RxNodeUpdate.NodeId,
        RxPackage->uReceiveParams.RxNodeUpdate.aPayload,
        RxPackage->uReceiveParams.RxNodeUpdate.iLength);
      break;
#if (defined(SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION) && SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION )
    case EZWAVERECEIVETYPE_REQUEST_ENCRYPTION_FRAME:
      ZW_RequestEncryptionStatus(request_protocol_cc_encryption(RxPackage) ? 
                                 ERPCCEEVENT_SERIALAPI_OK : ERPCCEEVENT_SERIALAPI_FAIL);
      break;
#endif
    default:
      break;
  }
}


/**
* @brief Triggered when protocol puts a message on the ZwCommandStatusQueue.
*/
void zaf_event_distributor_app_zw_command_status(SZwaveCommandStatusPackage *Status)
{
  DPRINTF("Incoming Status msg %x\r\n", Status->eStatusType);

  switch (Status->eStatusType) {
    case EZWAVECOMMANDSTATUS_LEARN_MODE_STATUS:
      SyncEventArg1Invoke(&LearnModeStatusCb, Status->Content.LearnModeStatus.Status);
      break;

    case EZWAVECOMMANDSTATUS_SET_DEFAULT:
      // Received when protocol is started (not implemented yet), and when SetDefault command is completed
      SyncEventInvoke(&SetDefaultCB);
      break;

#ifdef ZW_CONTROLLER
    case EZWAVECOMMANDSTATUS_REPLACE_FAILED_NODE_ID:
      ZCB_ComplHandler_ZW_ReplaceFailedNode(Status->Content.FailedNodeIDStatus.result);
      break;
    case EZWAVECOMMANDSTATUS_REMOVE_FAILED_NODE_ID:
      ZCB_ComplHandler_ZW_RemoveFailedNodeID(Status->Content.FailedNodeIDStatus.result);
      break;
    case EZWAVECOMMANDSTATUS_NETWORK_MANAGEMENT:
    {
      LEARN_INFO_T mLearnInfo;
      mLearnInfo.bStatus = Status->Content.NetworkManagementStatus.statusInfo[0];
      mLearnInfo.bSource = (uint16_t)(((uint16_t)Status->Content.NetworkManagementStatus.statusInfo[1] << 8)  // nodeID MSB
                                      | (uint16_t)Status->Content.NetworkManagementStatus.statusInfo[2]);     // nodeID LSB
      mLearnInfo.bLen = Status->Content.NetworkManagementStatus.statusInfo[3];
      mLearnInfo.pCmd = &Status->Content.NetworkManagementStatus.statusInfo[4];
      ZCB_ComplHandler_ZW_NodeManagement(&mLearnInfo);
      break;
    }
#if SUPPORT_ZW_SET_SLAVE_LEARN_MODE
    case EZWAVECOMMANDSTATUS_SET_SLAVE_LEARN_MODE:
    {
      uint8_t bStatus;
      uint16_t orgID;
      uint16_t newID;
      bStatus = Status->Content.NetworkManagementStatus.statusInfo[0];
      orgID = (uint16_t)((uint16_t)(Status->Content.NetworkManagementStatus.statusInfo[1] << 8)   // org nodeID MSB
                         | Status->Content.NetworkManagementStatus.statusInfo[2]);                // org nodeID LSB
      newID = (uint16_t)((uint16_t)(Status->Content.NetworkManagementStatus.statusInfo[3] << 8)   // new nodeID MSB
                         | Status->Content.NetworkManagementStatus.statusInfo[4]);                // new nodeID LSB
      ZCB_ComplHandler_ZW_SetSlaveLearnMode(bStatus, (uint8_t)orgID, (uint8_t)newID);            // orgID and newID are always (8-bit) IDs
      break;
    }
#endif
#endif
    default:
      break;
  }
}

static void
appFileSystemInit(void)
{
  SAppNodeInfo_t *AppNodeInfo;
  SRadioConfig_t *RadioConfig;

  AppNodeInfo = zaf_get_app_node_info();
  RadioConfig = zaf_get_radio_config();

  /*
   * Handle file system init inside Application Task
   * This reduces the default stack needed during initialization
   */
  if (SerialApiFileInit())
  {
    ReadApplicationSettings(&AppNodeInfo->DeviceOptionsMask, &AppNodeInfo->NodeType.generic, &AppNodeInfo->NodeType.specific);
    ReadApplicationCCInfo(&CommandClasses.UnSecureIncludedCC.iListLength,
                          (uint8_t*)CommandClasses.UnSecureIncludedCC.pCommandClasses,
                          &CommandClasses.SecureIncludedUnSecureCC.iListLength,
                          (uint8_t*)CommandClasses.SecureIncludedUnSecureCC.pCommandClasses,
                          &CommandClasses.SecureIncludedSecureCC.iListLength,
                          (uint8_t*)CommandClasses.SecureIncludedSecureCC.pCommandClasses);
    ReadApplicationRfRegion(&RadioConfig->eRegion);
    ReadApplicationTxPowerlevel(&RadioConfig->iTxPowerLevelMax, &RadioConfig->iTxPowerLevelAdjust);
    ReadApplicationMaxLRTxPwr(&RadioConfig->iTxPowerLevelMaxLR);
    ReadApplicationEnablePTI(&RadioConfig->radio_debug_enable);
    ReadApplicationNodeIdBaseType(&nodeIdBaseType);
  }
  else
  {
    /*
     * We end up here on the first boot after initializing the flash file system
     */

    zpal_radio_region_t mfgRegionConfig = REGION_UNDEFINED;
    // In case of valid MfgToken, override the app default settings.
    ZW_GetMfgTokenDataCountryFreq(&mfgRegionConfig);
    if (true == isRfRegionValid(mfgRegionConfig))
    {
      RadioConfig->eRegion = mfgRegionConfig;
    }

    // Save the setting to flash
    SaveApplicationRfRegion(RadioConfig->eRegion);
    // Save the default Tx powerlevel
    SaveApplicationTxPowerlevel(RadioConfig->iTxPowerLevelMax, RadioConfig->iTxPowerLevelAdjust);
    // write defualt values
    SaveApplicationSettings(AppNodeInfo->DeviceOptionsMask, AppNodeInfo->NodeType.generic, AppNodeInfo->NodeType.specific);
    // change the 20dBm tx power setting according to the application configuration
    SaveApplicationMaxLRTxPwr(RadioConfig->iTxPowerLevelMaxLR);

    SaveApplicationEnablePTI(RadioConfig->radio_debug_enable);
    SaveApplicationNodeIdBaseType(SERIAL_API_SETUP_NODEID_BASE_TYPE_DEFAULT);
  }

  ZAF_AppName_Write();
}
/*
 * The below function must be implemented as hardware specific function in a separate source
 * file if required.
 */
ZW_WEAK void SerialAPI_hw_psu_init(void)
{
  // Do nothing
}

/*===============================   ApplicationPoll   =======================
**    Application poll function, handling the receiving and transmitting
**    communication with the PC.
**
**--------------------------------------------------------------------------*/
static void           /*RET  Nothing                  */
ApplicationTask(SApplicationHandles* pAppHandles)
{
  uint32_t unhandledEvents = 0;
  SerialAPI_hw_psu_init(); // Must be invoked after the file system is initialized.

  // Init
  g_AppTaskHandle = xTaskGetCurrentTaskHandle();

  SetTaskHandle(g_AppTaskHandle);
  ZAF_setAppHandle(pAppHandles);
  ZW_system_startup_SetCCSet(&CommandClasses);

  AppTimerInit(EAPPLICATIONEVENT_TIMER, (void *) g_AppTaskHandle);
  radio_power_lock = zpal_pm_register(ZPAL_PM_TYPE_USE_RADIO);
  zpal_pm_stay_awake(radio_power_lock, 0);
  io_power_lock = zpal_pm_register(ZPAL_PM_TYPE_DEEP_SLEEP);
  zpal_pm_stay_awake(io_power_lock, 0);

  zaf_event_distributor_init();

  set_state_and_notify(stateStartup);
  // Wait for and process events
  DPRINT("SerialApi Event processor Started\r\n");
  for(;;) {
    unhandledEvents = zaf_event_distributor_distribute();
    if (0 != unhandledEvents) {
      DPRINTF("Unhandled Events: 0x%08lx\n", unhandledEvents);
#ifdef UNIT_TEST
      return;
#endif
    }
  }
}

static void SerialAPICommandHandler(void)
{
  if (bAwaitingConnection) {
    bAwaitingConnection = false;
    zaf_event_distributor_enqueue_proprietary_app_event(EVENT_APP_CONNECTED, NULL);
  }

  const bool handler_invoked = invoke_cmd_handler(serial_frame);
  if (!handler_invoked)
  {
    /* TODO - send a "Not Supported" respond frame */
    /* UNKNOWN - just drop it */
    set_state_and_notify(stateIdle);
  }
}


static void SerialAPIStateHandler(void)
{
  comm_interface_parse_result_t conVal;

  /* ApplicationPoll is controlled by a statemachine with the four states:
      stateIdle, stateFrameParse, stateTxSerial, stateCbTxSerial.

      stateIdle: If there is anything to transmit do so. -> stateCbTxSerial
                 If not, check if anything is received. -> stateFrameParse
                 If neither, stay in the state
                 Note: frames received while we are transmitting are lost
                 and must be retransmitted by PC

      stateFrameParse: Parse received frame.
                 If the request has no response -> stateIdle
                 If there is an immediate response send it. -> stateTxSerial

      stateTxSerial:  Waits for ack on responses send in stateFrameParse.
                 Retransmit frame as needed.
                 -> stateIdle

      stateCbTxSerial:  Waits for ack on requests send in stateIdle
                  (callback, ApplicationCommandHandler etc).
                 Retransmit frame as needed and remove from callbackqueue when done.
                 -> stateIdle

	  stateAppSuspend: Added for the uzb suspend function. The resume is through the suspend signal goes high in UZB stick
	                   The wakeup from deep sleep suspend causes system reboot

  */

  {
    switch (state)
    {
      case stateStartup:
        {
          ApplicationInitSW();
          SetRFReceiveMode(1);
          set_state_and_notify(stateIdle);
        }
        break;

      case stateIdle:
        {
          /* Check if there is anything to transmit. If so do it */
          if (callbackQueue.requestCnt)
          {
            comm_interface_transmit_frame(
              callbackQueue.requestQueue[callbackQueue.requestOut].wCmd,
              REQUEST,
              (uint8_t *)callbackQueue.requestQueue[callbackQueue.requestOut].wBuf,
              callbackQueue.requestQueue[callbackQueue.requestOut].wLen,
              NULL
            );
            set_state_and_notify(stateCallbackTxSerial);
            /* callbackCnt decremented when frame is acknowledged from PC - or timed out after retries */
          }
          else
          {
            /* Check if there is anything to transmit. If so do it */
            if (commandQueue.requestCnt)
            {
              comm_interface_transmit_frame(
                commandQueue.requestQueue[commandQueue.requestOut].wCmd,
                REQUEST,
                (uint8_t *)commandQueue.requestQueue[commandQueue.requestOut].wBuf,
                commandQueue.requestQueue[commandQueue.requestOut].wLen,
                NULL
              );
              set_state_and_notify(stateCommandTxSerial);
              /* commandCnt decremented when frame is acknowledged from PC - or timed out after retries */
            }
            else
            {
              /* Nothing to transmit. Check if we received anything */
              if (comm_interface_parse_data(true) == PARSE_FRAME_RECEIVED)
              {
                /* We got a frame... */
                set_state_and_notify(stateFrameParse);
              }
            }
          }
        }
        break;

    case stateFrameParse:
      {
        SerialAPICommandHandler();
      }
      break;

    case stateTxSerial:
    {
      /* Wait for ACK on send respond. Retransmit as needed */
      if ((conVal = comm_interface_parse_data(false)) == PARSE_FRAME_SENT)
      {
        /* One more RES transmitted succesfully */
        retry = 0;
        set_state_and_notify(stateIdle);
      }
      else if (conVal == PARSE_TX_TIMEOUT)
      {
        /* Either a NAK has been received or we timed out waiting for ACK */
        if (retry++ < MAX_SERIAL_RETRY)
        {
          comm_interface_transmit_frame(0, REQUEST, NULL, 0, NULL); /* Retry... */
        }
        else
        {
          /* Drop RES as HOST could not be reached */
          retry = 0;
          set_state_and_notify(stateIdle);
        }
      }
      /* All other states are ignored, as for now the only thing we are looking for is ACK/NAK! */
    }
    break;

    case stateCallbackTxSerial:
    {
      /* Wait for ack on unsolicited event (callback etc.) */
      /* Retransmit as needed. Remove frame from callbackQueue when done */
      if ((conVal = comm_interface_parse_data(false)) == PARSE_FRAME_SENT)
      {
        /* One more REQ transmitted succesfully */
        PopCallBackQueue();
      }
      else if (conVal == PARSE_TX_TIMEOUT)
      {
        /* Either a NAK has been received or we timed out waiting for ACK */
        if (retry++ < MAX_SERIAL_RETRY)
        {
          comm_interface_transmit_frame(0, REQUEST, NULL, 0, NULL); /* Retry... */
        }
        else
        {
          /* Drop REQ as HOST could not be reached */
          PopCallBackQueue();
        }
      }
      /* All other states are ignored, as for now the only thing we are looking for is ACK/NAK! */
    }
    break;

    case stateCommandTxSerial:
    {
      /* Wait for ack on unsolicited ApplicationCommandHandler event */
      /* Retransmit as needed. Remove frame from comamndQueue when done */
      if ((conVal = comm_interface_parse_data(false)) == PARSE_FRAME_SENT)
      {
        /* One more REQ transmitted succesfully */
        PopCommandQueue();
      }
      else if (conVal == PARSE_TX_TIMEOUT)
      {
        /* Either a NAK has been received or we timed out waiting for ACK */
        if (retry++ < MAX_SERIAL_RETRY)
        {
          comm_interface_transmit_frame(0, REQUEST, NULL, 0, NULL); /* Retry... */
        }
        else
        {
          /* Drop REQ as HOST could not be reached */
          PopCommandQueue();
        }
      }
      /* All other states are ignored, as for now the only thing we are looking for is ACK/NAK! */
    }
    break;
    default:
      set_state_and_notify(stateIdle);
      break;
    }

  } // For loop - task loop
}


void
zaf_event_distributor_app_state_change(void)
{
  SerialAPIStateHandler();
}

void
zaf_event_distributor_app_serial_data_rx(void)
{
  SerialAPIStateHandler();
}

void
zaf_event_distributor_app_serial_timeout(void)
{
  SerialAPIStateHandler();
}

void
PopCallBackQueue(void)
{
  if (callbackQueue.requestCnt)
  {
    callbackQueue.requestCnt--;
    if (++callbackQueue.requestOut >= MAX_CALLBACK_QUEUE)
    {
      callbackQueue.requestOut = 0;
    }
  }
  else
  {
    callbackQueue.requestOut = callbackQueue.requestIn;
  }
  retry = 0;
  set_state_and_notify(stateIdle);
}


void
PopCommandQueue(void)
{
  if (commandQueue.requestCnt)
  {
    commandQueue.requestCnt--;
    if (++commandQueue.requestOut >= MAX_UNSOLICITED_QUEUE)
    {
      commandQueue.requestOut = 0;
    }
  }
  else
  {
    commandQueue.requestOut = commandQueue.requestIn;
  }
  retry = 0;
  set_state_and_notify(stateIdle);
}

/**
 * @brief wakeup after sleep timeout event
 *
 * @param pTimer Timer connected to this method
 */
void
ZCB_WakeupTimeout(__attribute__((unused)) SSwTimer *pTimer)
{
  DPRINT("ZCB_WakeupTimeout\n");
}

/* Returns the current effect that should be used for the LED */
LedEffect_t*
get_current_led_effect(void) {
  if (ledEffectSystem.type != LED_EFFECT_NOT_SET) {
    return &ledEffectSystem;
  } else if (bEnableTiltDetection && bTiltDetected) {
    return &ledEffectTilt;
  } else if (ledEffectUser.type != LED_EFFECT_NOT_SET) {
    return &ledEffectUser;
  } else {
    return &ledEffectDefault;
  }
}

void
trigger_led_effect_refresh(void) {
  if (ledEffectUser.type == LED_EFFECT_SOLID) {
    ledEffectUser.effect.solid.modified = true;
  }
  if (ledEffectDefault.type == LED_EFFECT_SOLID) {
    ledEffectDefault.effect.solid.modified = true;
  }
}

void
zaf_event_distributor_app_proprietary(event_nc_t *event)
{
  // Handles NC-specific proprietary events
  EVENT_APP event_nc = (EVENT_APP) event->event;
  switch (event_nc) {
    case EVENT_APP_USERTASK_READY: {
      // Fade slowly to white
      LedEffectFade_t fade = {
        .color = cold_white,
        .brightness = FADE_MAX_BRIGHTNESS,
        .increasing = false,
        .ticksPerStep = 2,
        .stepSize = 1,
        .tickCounter = 0
      };
      ledEffectDefault = (LedEffect_t) {
        .type = LED_EFFECT_FADE,
        .effect.fade = fade
      };
      bAwaitingConnection = true;
      break;
    }

    case EVENT_APP_CONNECTED: {
      if (ledEffectDefault.type == LED_EFFECT_FADE) {
        // Stop the animation to indicate that we're connected
        ledEffectDefault.effect.fade.stopAtMax = true;
      } else {
        // If we were not in fade mode, set the color to white
        LedEffectSolid_t solid = {
          .color = cold_white,
          .modified = true
        };
        ledEffectDefault = (LedEffect_t) {
          .type = LED_EFFECT_SOLID,
          .effect.solid = solid
        };
      }
      break;
    }

    case EVENT_APP_USERTASK_GYRO_MEASUREMENT: {
      // A gyro measurement was requested
      gyro_reading_t gyro_reading = event->payload->gyro_reading;

      // Debounce the readings
      if (last_gyro_reading.x == 0 && last_gyro_reading.y == 0 && last_gyro_reading.z == 0) {
        last_gyro_reading = gyro_reading;
        return;
      } else if (abs(gyro_reading.x - last_gyro_reading.x) < 25 &&
                 abs(gyro_reading.y - last_gyro_reading.y) < 25 &&
                 abs(gyro_reading.z - last_gyro_reading.z) < 25) {
        stable_gyro_readings++;
        // Prevent overflows
        if (stable_gyro_readings > 0xffff) {
          stable_gyro_readings = GYRO_STABLE_THRESHOLD;
        }
      } else {
        stable_gyro_readings = 0;
      }
      last_gyro_reading = gyro_reading;

      if (stable_gyro_readings < GYRO_STABLE_THRESHOLD) {
        // Not enough stable readings, ignore this one
        return;
      }

      // Compute the angle from vertical (0,0,-1)
      float gyro_magnitude = sqrtf(
        gyro_reading.x * gyro_reading.x +
        gyro_reading.y * gyro_reading.y +
        gyro_reading.z * gyro_reading.z
      );
      // horizontal components of the reference vector are 0
      float dot_product = (float) -gyro_reading.z;
      float angle = acos(dot_product / gyro_magnitude) * 180.0f / M_PI; // Convert to degrees

      // The angle will be between 0 and 180°, accept everything that's close
      // enough to those values

      // Indicate bad orientation (more than 20° from vertical) in calibration mode
      if (bTiltDetected && (angle < TILT_THRESHOLD_LOWER || angle > (180.0f - TILT_THRESHOLD_LOWER))) {
        // Tilt no longer detected when crossing the upper threshold
        bTiltDetected = false;
        // Mark the user LED effect as modified, so it gets used again
        trigger_led_effect_refresh();
      } else if (!bTiltDetected && angle > TILT_THRESHOLD_UPPER && angle < (180.0f - TILT_THRESHOLD_UPPER)) {
        // Tilt no longer detected when crossing the lower threshold
        bTiltDetected = true;
        // Indicate incorrect tilt using the LED
        LedEffectBlink_t blink = {
          .color = cold_white,
          .levelBright = FADE_MAX_BRIGHTNESS,
          .levelDim = FADE_MIN_BRIGHTNESS,
          // 1 tick = 2ms, blink 250ms on, 250ms off
          .ticksBright = 125,
          .ticksDim = 125,
          .tickCounter = 0,
          .bright = false,
        };
        ledEffectTilt = (LedEffect_t) {
          .type = LED_EFFECT_BLINK,
          .effect.blink = blink
        };
      }

      if (!bRequestGyroMeasurement) {
        return;
      }
      bRequestGyroMeasurement = false;

      uint8_t cmd[8];
      uint8_t i=0;
      cmd[i++] = NABU_CASA_GYRO_MEASURE;
      cmd[i++] = gyro_reading.x >> 8;
      cmd[i++] = gyro_reading.x & 0xFF;
      cmd[i++] = gyro_reading.y >> 8;
      cmd[i++] = gyro_reading.y & 0xFF;
      cmd[i++] = gyro_reading.z >> 8;
      cmd[i++] = gyro_reading.z & 0xFF;
      RequestUnsolicited(
        FUNC_ID_NABU_CASA,
        cmd,
        i
      );
      break;
    }

    case EVENT_APP_USERTASK_TICK_LED: {
      LedEffect_t* ledEffect = get_current_led_effect();

      switch (ledEffect->type) {
        case LED_EFFECT_SOLID: {
          // For solid LED effect, set the color once
          if (ledEffect->effect.solid.modified) {
            ledEffect->effect.solid.modified = false;
            set_color_buffer(ledEffect->effect.solid.color);
          }
          break;
        }

        case LED_EFFECT_FADE: {
          // For fading, change the brightness every N ticks,
          // and update the color
          LedEffectFade_t fade = ledEffect->effect.fade;

          if (fade.brightness >= FADE_BRIGHT_THRESHOLD && fade.stopAtMax) {
            // Switch to solid mode
            LedEffectSolid_t solid = {
              .color = fade.color,
              .modified = true
            };
            *ledEffect = (LedEffect_t) {
              .type = LED_EFFECT_SOLID,
              .effect.solid = solid
            };
            break;
          }

          if (fade.tickCounter == 0) {
            if (fade.increasing) {
              uint8_t delta = min(fade.stepSize, FADE_MAX_BRIGHTNESS - fade.brightness);
              fade.brightness += delta;
              if (fade.brightness == FADE_MAX_BRIGHTNESS) {
                fade.increasing = false;
              }
            } else {
              uint8_t delta = min(fade.stepSize, fade.brightness - FADE_MIN_BRIGHTNESS);
              fade.brightness -= delta;
              if (fade.brightness == FADE_MIN_BRIGHTNESS) {
                fade.increasing = true;
              }
            }

            uint16_t r = ((uint16_t) fade.color.R) * ((uint16_t) fade.brightness) / FADE_MAX_BRIGHTNESS;
            uint16_t g = ((uint16_t) fade.color.G) * ((uint16_t) fade.brightness) / FADE_MAX_BRIGHTNESS;
            uint16_t b = ((uint16_t) fade.color.B) * ((uint16_t) fade.brightness) / FADE_MAX_BRIGHTNESS;

            rgb_t color = {
              (uint8_t) g,
              (uint8_t) r,
              (uint8_t) b
            };

            set_color_buffer(color);
          }

          fade.tickCounter = (fade.tickCounter + 1) % fade.ticksPerStep;
          ledEffect->effect.fade = fade;
          break;
        }

        case LED_EFFECT_BLINK: {
          // For blinking, switch between min and max brightness
          // and keep the brightness for the given number of ticks
          LedEffectBlink_t blink = ledEffect->effect.blink;

          if (blink.tickCounter == 0) {
            blink.bright = !blink.bright;
            uint8_t level = blink.bright ? blink.levelBright : blink.levelDim;

            uint16_t r = ((uint16_t) blink.color.R) * ((uint16_t) level) / FADE_MAX_BRIGHTNESS;
            uint16_t g = ((uint16_t) blink.color.G) * ((uint16_t) level) / FADE_MAX_BRIGHTNESS;
            uint16_t b = ((uint16_t) blink.color.B) * ((uint16_t) level) / FADE_MAX_BRIGHTNESS;

            rgb_t color = {
              (uint8_t) g,
              (uint8_t) r,
              (uint8_t) b
            };

            set_color_buffer(color);
          }

          uint8_t maxTicks = blink.bright ? blink.ticksBright : blink.ticksDim;
          blink.tickCounter = (blink.tickCounter + 1) % maxTicks;
          ledEffect->effect.blink = blink;
          break;
        }

        default:
          // Nothing to do
          break;
      } // switch (ledEffect->type)
      break;
    }

    default:
      // Nothing to do
      break;
  } // switch (event_nc)
}

// Called when the button next to the USB port is pressed or released
// void sl_button_on_change(const sl_button_t *handle)
// {
  // if (handle->get_state(handle)) {
  //   rgb_t color = {255, 0, 0};
  //   set_color_buffer(color);
  // } else {
  //   rgb_t color = {4, 0, 0};
  //   set_color_buffer(color);
  // }
// }

/*==============================   ApplicationInitSW   ======================
**    Initialization of the Application Software
**
**--------------------------------------------------------------------------*/
void
ApplicationInitSW(void)
{
  SAppNodeInfo_t *AppNodeInfo;
  SRadioConfig_t *RadioConfig;

  AppNodeInfo = zaf_get_app_node_info();
  RadioConfig = zaf_get_radio_config();

  comm_interface_init();

  // FIXME load any saved node configuration and prepare to feed it to protocol
/* Do we together with the bTxStatus uint8_t also transmit a sTxStatusReport struct on ZW_SendData callback to HOST */
#if SUPPORT_SEND_DATA_TIMING
  bTxStatusReportEnabled = true;
#else
  bTxStatusReportEnabled = false;
#endif

#if SUPPORT_SERIAL_API_STARTUP_NOTIFICATION
  /* ZW->HOST: bWakeupReason | bWatchdogStarted | deviceOptionMask | */
  /*           nodeType_generic | nodeType_specific | cmdClassLength | cmdClass[] */

  // FIXME send startup notification via serial port if we are supposed to
  SCommandClassList_t *const apCCLists[3] =
  {
    &CommandClasses.UnSecureIncludedCC,
    &CommandClasses.SecureIncludedUnSecureCC,
    &CommandClasses.SecureIncludedSecureCC
  };

  compl_workbuf[0] = g_eApplResetReason;
#if SUPPORT_ZW_WATCHDOG_START || SUPPORT_ZW_WATCHDOG_STOP
  compl_workbuf[1] = bWatchdogStarted;
#else
  compl_workbuf[1] = false;
#endif
  compl_workbuf[2] = AppNodeInfo->DeviceOptionsMask;
  compl_workbuf[3] = AppNodeInfo->NodeType.generic;
  compl_workbuf[4] = AppNodeInfo->NodeType.specific;
  compl_workbuf[5] = apCCLists[0]->iListLength;
  uint8_t i = 0;
  if (0 < apCCLists[0]->iListLength)
  {
    for (i = 0; i < apCCLists[0]->iListLength; i++)
    {
      compl_workbuf[6 + i] = apCCLists[0]->pCommandClasses[i];
    }
  }

  eSerialAPIStartedCapabilities capabilities = 0;
  if (ZAF_isLongRangeRegion(RadioConfig->eRegion))
      capabilities = SERIAL_API_STARTED_CAPABILITIES_L0NG_RANGE;
  compl_workbuf[6 + i] = capabilities;

  uint32_t zpal_reset_info = 0;
  if (ZPAL_STATUS_OK != zpal_retention_register_read(ZPAL_RETENTION_REGISTER_RESET_INFO, &zpal_reset_info))
  {
    DPRINT("ERROR while reading the reset information\n");
  Request(FUNC_ID_SERIAL_API_STARTED, compl_workbuf, 7 + i);
  }
  else
  {
    compl_workbuf[7 + i] = (uint8_t)(zpal_reset_info >> 24);
    compl_workbuf[8 + i] = (uint8_t)(zpal_reset_info >> 16);
    compl_workbuf[9 + i] = (uint8_t)(zpal_reset_info >> 8);
    compl_workbuf[10 + i] = (uint8_t)zpal_reset_info;
    DPRINTF("zpal_reset_reason: %u\n", zpal_reset_info);
    Request(FUNC_ID_SERIAL_API_STARTED, compl_workbuf, 11 + i);
  }

#endif /* #if SUPPORT_STARTUP_NOTIFICATION */
   AppTimerDeepSleepPersistentRegister(&mWakeupTimer, false, ZCB_WakeupTimeout);  // register for event jobs timeout event
}


/*==============================   ApplicationInit   ======================
**    Init UART and setup port pins for LEDs
**
**--------------------------------------------------------------------------*/
ZW_APPLICATION_STATUS
ApplicationInit(
  zpal_reset_reason_t eResetReason)
{
  // enable the watchdog at init of application
  zpal_enable_watchdog(true);
  
  // Serial API can control hardware with information
  // set in the file system therefore it should be the first
  // step in the Initialization
  appFileSystemInit();

#if (!defined(SL_CATALOG_SILICON_LABS_ZWAVE_APPLICATION_PRESENT) && !defined(UNIT_TEST))
  /* This preprocessor statement can be deleted from the source code */
  app_hw_init();
#endif

  /* g_eApplResetReason now contains lastest System Reset reason */
  g_eApplResetReason = eResetReason;

#ifdef DEBUGPRINT
  static uint8_t m_aDebugPrintBuffer[96];
  DebugPrintConfig(m_aDebugPrintBuffer, sizeof(m_aDebugPrintBuffer), zpal_debug_output);
  DebugPrintf("ApplicationInit eResetReason = %d\n", eResetReason);
  ZAF_PrintAppInfo();
#endif

  // Initialize NC-specific hardware
  initWs2812();
  initqma6100p();

  // Work around bugs in SDK 7.23.0/1 where the protocol file version is not written to NVM
  // TODO: Remove in 7.23.2 where this is fixed
  zpal_nvm_handle_t stack_handle = zpal_nvm_init(ZPAL_NVM_AREA_STACK);
  if (stack_handle != NULL) {
    size_t len = 0;
    zpal_nvm_get_object_size(stack_handle, 0x0, &len);
    if (len == 0) {
      // The protocol file version is not written to NVM, so write it
      uint32_t protocol_file_version = 0x05071701; // 7.23.1, format 5
      zpal_nvm_write(stack_handle, 0x0, &protocol_file_version, sizeof(protocol_file_version));
    }
  }

  // Try to restore the user-defined LED state from NVM. In the first revision this was stored
  // as RGB values, but now we only care about on/off
  NabuCasaLedStorage_t ledStorage = {0};
  if (
    ZPAL_STATUS_OK == ZAF_nvm_app_read(FILE_ID_NABUCASA_LED, &ledStorage, sizeof(ledStorage))
    && ledStorage.valid
  ) {
    bool state = ledStorage.r > black.R || ledStorage.g > black.G || ledStorage.b > black.B;
    rgb_t color = state ? cold_white : black;
    ledEffectUser = (LedEffect_t) {
      .type = LED_EFFECT_SOLID,
      .effect.solid = {
        .color = color,
        .modified = true
      }
    };
    set_color_buffer(color);
  } else {
    // Default to on
    set_color_buffer(cold_white);
  }

  // Try to restore configuration settings from NVM
  bEnableTiltDetection = nc_config_get(NC_CFG_ENABLE_TILT_INDICATOR);

  /*************************************************************************************
   * CREATE USER TASKS  -  ZW_ApplicationRegisterTask() and ZW_UserTask_CreateTask()
   *************************************************************************************
   * Register the main APP task function.
   *
   * ATTENTION: This function is the only task that can call ZAF API functions!!!
   * Failure to follow guidelines will result in undefined behavior.
   *
   * Furthermore, this function is the only way to register Event Notification
   * Bit Numbers for associating to given event handlers.
   *
   * ZW_UserTask_CreateTask() can be used to create additional tasks.
   * @see zwave_soc_sensor_pir example for more info.
   *************************************************************************************/
  __attribute__((unused)) bool bWasTaskCreated = ZW_ApplicationRegisterTask(
                                                    ApplicationTask,
                                                    EAPPLICATIONEVENT_ZWRX,
                                                    EAPPLICATIONEVENT_ZWCOMMANDSTATUS,
                                                    zaf_get_protocol_config()
                                                    );
  assert(bWasTaskCreated);


  // Interact with the hardware in a background task
  ZW_UserTask_Buffer_t bgHwTaskBuffer;
  bgHwTaskBuffer.taskBuffer = &BackgroundHwTaskBuffer;
  bgHwTaskBuffer.stackBuffer = BackgroundHwStackBuffer;
  bgHwTaskBuffer.stackBufferLength = HW_TASK_STACK_SIZE;

  // Create the task setting-structure!
  ZW_UserTask_t task;
  task.pTaskFunc = (TaskFunction_t)NC_UserTask_Hardware;
  task.pTaskName = "DataAcqu";
  task.pUserTaskParam = NULL;  // We pass nothing here, as the EventHelper is already initialized and can be used for task IPC!
  task.priority = USERTASK_PRIORITY_NORMAL;
  task.taskBuffer = &bgHwTaskBuffer;

  // Create the task!
  ZW_UserTask_CreateTask(&task, &m_xTaskHandleBackgroundHw);

  return (APPLICATION_RUNNING); /*Return false to enter production test mode*/
}


#ifndef ZW_CONTROLLER_BRIDGE
/*==========================   ApplicationCommandHandler   ==================
**    Handling of received application commands and requests
**
**--------------------------------------------------------------------------*/
void /*RET Nothing                  */
ApplicationCommandHandler(__attribute__((unused)) void *pSubscriberContext, SZwaveReceivePackage* pRxPackage)
{
  ZW_APPLICATION_TX_BUFFER *pCmd = (ZW_APPLICATION_TX_BUFFER *)&pRxPackage->uReceiveParams.Rx.Payload;
  uint8_t cmdLength = pRxPackage->uReceiveParams.Rx.iLength;
  RECEIVE_OPTIONS_TYPE *rxOpt = &pRxPackage->uReceiveParams.Rx.RxOptions;
  /* ZW->PC: REQ | 0x04 | rxStatus | sourceNode | cmdLength | pCmd[] | rssiVal | securityKey */
  uint8_t offset = 0;
  compl_workbuf[0] = rxOpt->rxStatus;
  if (SERIAL_API_SETUP_NODEID_BASE_TYPE_16_BIT == nodeIdBaseType)
  {
    compl_workbuf[1] = (uint8_t)(rxOpt->sourceNode >> 8);     // MSB
    compl_workbuf[2] = (uint8_t)(rxOpt->sourceNode & 0xFF);   // LSB
    offset++;  // 16 bit nodeID means the command fields that follow are offset by one byte
  }
  else
  {
    compl_workbuf[1] = (uint8_t)(rxOpt->sourceNode & 0xFF);       // Legacy 8 bit nodeID
  }
  if (cmdLength > (uint8_t)(BUF_SIZE_TX - (offset + 5)))
  {
    cmdLength = (uint8_t)(BUF_SIZE_TX - (offset + 5));
  }
  compl_workbuf[offset + 2] = cmdLength;
  for (uint8_t i = 0; i < cmdLength; i++)
  {
    compl_workbuf[offset + 3 + i] = *((uint8_t*)pCmd + i);
  }
  /* Syntax when a promiscuous frame is received (i.e. RECEIVE_STATUS_FOREIGN_FRAME is set): */
  /* ZW->PC: REQ | 0xD1 | rxStatus | sourceNode | cmdLength | pCmd[] | destNode | rssiVal
   * | securityKey | bSourceTxPower | bSourceNoiseFloor */
  compl_workbuf[offset + 3 + cmdLength] = (uint8_t)rxOpt->rxRSSIVal;
  compl_workbuf[offset + 4 + cmdLength] = rxOpt->securityKey;
  compl_workbuf[offset + 5 + cmdLength] = (uint8_t)rxOpt->bSourceTxPower;
  compl_workbuf[offset + 6 + cmdLength] = (uint8_t)rxOpt->bSourceNoiseFloor;

  /* Less code space-consuming version for libraries without promiscuous support */
  RequestUnsolicited(FUNC_ID_APPLICATION_COMMAND_HANDLER, compl_workbuf, (uint8_t)(offset + 7 + cmdLength));
}
#endif

#ifdef ZW_CONTROLLER_BRIDGE

// Struct describing Multicast nodemask header for SerialAPI
typedef struct SMultiCastNodeMaskHeaderSerial
{
  uint8_t iNodemaskLength : 5; // Bits 0-4 is length. Length of Nodemask in bytes - Valid values [0-29]
  uint8_t iNodeMaskOffset : 3; // Bits 5-7 is offset. Denotes which node the first bit in the nodemask describes
                               // First node in nodemask is (Value * 32) + 1 - e.g. 2 -> first node is 65
                               // In reality - we always give a full nodemask -> length 29, offset 0.
} SMultiCastNodeMaskHeaderSerial;

/*======================   ApplicationCommandHandler_Bridge   ================
**    Handling of received application commands and requests
**
**--------------------------------------------------------------------------*/
static void                       /*RET Nothing                  */
ApplicationCommandHandler_Bridge(SReceiveMulti* pReceiveMulti)
{
  /* ZW->HOST: REQ | 0xA8 | rxStatus | destNode | sourceNode | cmdLength
   *          | pCmd[] | multiDestsOffset_NodeMaskLen | multiDestsNodeMask[] | rssiVal
   *          | securityKey | bSourceTxPower | bSourceNoiseFloor */
  uint8_t offset = 0;
  compl_workbuf[0] = pReceiveMulti->RxOptions.rxStatus;
  if (SERIAL_API_SETUP_NODEID_BASE_TYPE_16_BIT == nodeIdBaseType)
  {
    compl_workbuf[1] = (uint8_t)(pReceiveMulti->RxOptions.destNode >> 8);      // MSB
    compl_workbuf[2] = (uint8_t)(pReceiveMulti->RxOptions.destNode & 0xFF);    // LSB
    compl_workbuf[3] = (uint8_t)(pReceiveMulti->RxOptions.sourceNode >> 8);    // MSB
    compl_workbuf[4] = (uint8_t)(pReceiveMulti->RxOptions.sourceNode & 0xFF);  // LSB
    offset = 6;  // 16 bit nodeIDs means the command fields that follow are offset by two bytes
  }
  else
  {
    // Legacy 8 bit nodeIDs
    compl_workbuf[1] = (uint8_t)pReceiveMulti->RxOptions.destNode;
    compl_workbuf[2] = (uint8_t)pReceiveMulti->RxOptions.sourceNode;
    offset = 4;
  }

  uint32_t cmdLength = pReceiveMulti->iCommandLength;
  uint8_t i;

  if (cmdLength > sizeof(pReceiveMulti->Payload))
  {
    cmdLength = sizeof(pReceiveMulti->Payload);
  }
  if (cmdLength > (uint8_t)(BUF_SIZE_TX - offset) )
  {
    cmdLength = (uint8_t)(BUF_SIZE_TX - offset) ;
  }
  compl_workbuf[offset - 1 ] = (uint8_t)cmdLength;

  memcpy(compl_workbuf + offset, (uint8_t*)&pReceiveMulti->Payload, cmdLength);

  if (pReceiveMulti->RxOptions.rxStatus & RECEIVE_STATUS_TYPE_MULTI)
  {
    /* Its a Multicast frame */

    // Create NodeMaskHeader to comply with SerialAPI
    const SMultiCastNodeMaskHeaderSerial NodeMaskHeader = {
        .iNodemaskLength = 29, // Always offset full nodemask. Hardwired to 29 (and not
                               // nodemask define) since SerialAPI is not supposed to change.
        .iNodeMaskOffset = 0   // Always full nodemask -> No offset
    };

    i = NodeMaskHeader.iNodemaskLength + 1; // + 1 for node mask headers own size.
    if (i > (uint8_t)(BUF_SIZE_TX - (offset + cmdLength)))
    {
      i = (uint8_t)(BUF_SIZE_TX - (offset + cmdLength + 1));
    }
    if (i > 0)
    {
      *(compl_workbuf + offset + cmdLength) = i - 1;
      memcpy(compl_workbuf + offset + 1 + cmdLength, (uint8_t*)pReceiveMulti->NodeMask, i - 1); // +- 1 as we already copied node mask header
      i += (uint8_t)cmdLength;
    }
  }
  else
  {
    if (cmdLength >= (uint8_t)(BUF_SIZE_TX - offset) )
    {
      cmdLength = (uint8_t)(BUF_SIZE_TX - offset -1);
      i = (uint8_t)cmdLength;
    }
    else
    {
      i = (uint8_t)(cmdLength + 1);
    }
    compl_workbuf[(uint8_t)(offset + cmdLength)] = 0;
  }
  compl_workbuf[offset + i] = (uint8_t)pReceiveMulti->RxOptions.rxRSSIVal;
  if (SERIAL_API_SETUP_NODEID_BASE_TYPE_16_BIT == nodeIdBaseType)
  {
    compl_workbuf[offset + ++i] = pReceiveMulti->RxOptions.securityKey; //inclusion fails without this
    compl_workbuf[offset + ++i] = (uint8_t)pReceiveMulti->RxOptions.bSourceTxPower;
    compl_workbuf[offset + ++i] = (uint8_t)pReceiveMulti->RxOptions.bSourceNoiseFloor;
  }
  /* Unified Application Command Handler for Bridge and Virtual nodes */
  RequestUnsolicited(FUNC_ID_APPLICATION_COMMAND_HANDLER_BRIDGE, compl_workbuf, (uint8_t)(offset + 1 + i));
}
#endif

#if (defined(SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION) && SUPPORT_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION )
bool request_protocol_cc_encryption(SZwaveReceivePackage *pRPCCEPackage)
{
  bool status = false;
  ZW_APPLICATION_TX_BUFFER *pCmd = (ZW_APPLICATION_TX_BUFFER *)&pRPCCEPackage->uReceiveParams.RequestEncryption.Payload;
  uint8_t cmdLength = pRPCCEPackage->uReceiveParams.RequestEncryption.payloadLength;
  uint8_t *protocolMetadata = (uint8_t *) &pRPCCEPackage->uReceiveParams.RequestEncryption.protocolMetadata;
  uint8_t protocolMetadataLength = pRPCCEPackage->uReceiveParams.RequestEncryption.protocolMetadataLength;
  node_id_t destNodeID = pRPCCEPackage->uReceiveParams.RequestEncryption.destNodeID;
  uint8_t useSupervision = pRPCCEPackage->uReceiveParams.RequestEncryption.useSupervision;
  static uint8_t sessionId = 0;

  if (protocolMetadataLength != PROTOCOL_METADATA_LENGTH ||
      cmdLength > (uint8_t)(BUF_SIZE_TX - (5 + protocolMetadataLength)))
  {
    return status;
  }

  /*ZW->HOST: REQ | 0x6C | destNodeID | cmdLength | pCmd | protocolMetadataLength | protocolMetadata | Use Supervision | SessionID */
  uint8_t offset = 0;
  if (SERIAL_API_SETUP_NODEID_BASE_TYPE_16_BIT == nodeIdBaseType)
  {
    compl_workbuf[0] = (uint8_t) (destNodeID >> 8);     // MSB
    compl_workbuf[1] = (uint8_t) (destNodeID & 0xFF);   // LSB
    offset += 2;  // 16 bit nodeID means the command fields that follow are offset by one byte
  }
  else
  {
    compl_workbuf[0] = (uint8_t) (destNodeID & 0xFF);       // Legacy 8 bit nodeID
    offset++;
  }

  compl_workbuf[offset++] = cmdLength;
  memcpy(&compl_workbuf[offset], pCmd, cmdLength);
  offset += cmdLength;

  compl_workbuf[offset++] = protocolMetadataLength;
  memcpy(&compl_workbuf[offset], protocolMetadata, protocolMetadataLength);
  offset += protocolMetadataLength;

  compl_workbuf[offset++] = useSupervision;
  compl_workbuf[offset++] = sessionId;

  sessionId = (uint8_t) (sessionId % 255) + 1;
  
  status = RequestUnsolicited(FUNC_ID_ZW_REQUEST_PROTOCOL_CC_ENCRYPTION, compl_workbuf, offset);
  return status;
}
#endif

/*======================   ApplicationNodeUpdate   =========================
**    Inform the static controller/slave of node information received
**
**--------------------------------------------------------------------------*/
void                                /* RET  Nothing                         */
ApplicationNodeUpdate(
  uint8_t bStatus,                     /* IN   Status of learn mode            */
  uint16_t nodeID,                    /* IN   Node id of node sending nodeinfo*/
  uint8_t *pCmd,                       /* IN   Pointer to appl. node info      */
  uint8_t bLen                         /* IN   Node info length                */
)
{
  uint8_t offset = 0;
  compl_workbuf[0] = bStatus;
  if (SERIAL_API_SETUP_NODEID_BASE_TYPE_16_BIT == nodeIdBaseType)
  {
    compl_workbuf[1] = (uint8_t)(nodeID >> 8);     // MSB
    compl_workbuf[2] = (uint8_t)(nodeID & 0xFF);   // LSB
    offset++;  // 16 bit nodeID means the command fields that follow are offset by one byte
  }
  else
  {
    compl_workbuf[1] = (uint8_t)(nodeID & 0xFF);      // Legacy 8 bit nodeID
  }

  /*  - Buffer boundary check */
  bLen = (bLen > MAX_NODE_INFO_LENGTH) ? MAX_NODE_INFO_LENGTH : bLen;
  bLen = (bLen > (uint8_t)(BUF_SIZE_TX - (offset + 3))) ? (uint8_t)(BUF_SIZE_TX - (offset + 3)) : bLen;
  
  compl_workbuf[offset + 2] = bLen;
  if(bLen > 0 && pCmd)
  {
    for (uint8_t i = 0; i < bLen; i++)
    {
      compl_workbuf[offset + 3 + i] = *(pCmd + i);
    }
  }
  RequestUnsolicited(FUNC_ID_ZW_APPLICATION_UPDATE, compl_workbuf, (uint8_t)(offset + bLen + 3));
}

ZW_WEAK const void * SerialAPI_get_uart_config_ext(void)
{
  return NULL;
}
