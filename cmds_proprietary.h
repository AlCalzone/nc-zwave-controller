
#ifndef APPS_SERIALAPI_CMD_PROPRIETARY_H_
#define APPS_SERIALAPI_CMD_PROPRIETARY_H_

#include <stdint.h>
#include <ZAF_types.h>

#define FUNC_ID_NABU_CASA FUNC_ID_PROPRIETARY_0

/* FUNC_ID_PROPRIETARY_0 (Nabu Casa) command definitions */
typedef enum
{
  NABU_CASA_CMD_SUPPORTED = 0,
  NABU_CASA_LED_GET = 1,
  NABU_CASA_LED_SET = 2,
  NABU_CASA_GYRO_MEASURE = 3,
  NABU_CASA_SYSTEM_INDICATION_SET = 4,
} eNabuCasaCmd;

typedef enum
{
  NC_LED_FX_NOT_SET = 0,
  NC_LED_FX_SOLID = 1,
  NC_LED_FX_FADE = 2,
} eNabuCasaLedEffect;

typedef enum NC_SYS_INDICATION
{
  NC_SYS_INDICATION_OFF = 0,
  NC_SYS_INDICATION_WARN = 1,
  NC_SYS_INDICATION_ERROR = 2,
} eNabuCasaSystemIndication;

// Proprietary data storage
#define NC_APPDATA_OFFSET_LED 0x00

typedef struct __attribute__((packed)) NabuCasaLedStorage
{
  bool valid;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} NabuCasaLedStorage_t;

#endif /* APPS_SERIALAPI_CMD_PROPRIETARY_H_ */
