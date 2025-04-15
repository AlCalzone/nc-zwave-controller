
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
  NABU_CASA_CONFIG_GET = 5,
  NABU_CASA_CONFIG_SET = 6,
} eNabuCasaCmd;

typedef enum
{
  NC_LED_FX_NOT_SET = 0,
  NC_LED_FX_SOLID = 1,
  NC_LED_FX_FADE = 2,
} eNabuCasaLedEffect;

typedef enum
{
  NC_SYS_INDICATION_OFF = 0,
  NC_SYS_INDICATION_WARN = 1,
  NC_SYS_INDICATION_ERROR = 2,
} eNabuCasaSystemIndication;

// Proprietary data storage
// Application specific files should use identifiers in the range 0x00000—0x0FFFF,
// so the effective identifier address range is reduced to 16 bits.
#define FILE_ID_NABUCASA_LED  0x4660
#define FILE_ID_NABUCASA_CONFIG 0x4661

typedef struct __attribute__((packed)) NabuCasaLedStorage
{
  bool valid;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} NabuCasaLedStorage_t;

typedef enum
{
  NC_CFG_ENABLE_TILT_INDICATOR = 0,
} eNabuCasaConfigKey;

typedef enum
{
  NC_CFG_FLAG_ENABLE_TILT_INDICATOR = (1 << NC_CFG_ENABLE_TILT_INDICATOR),
} eNabuCasaConfigFlags;

typedef struct __attribute__((packed)) NabuCasaConfigStorage
{
  eNabuCasaConfigFlags flags;
} NabuCasaConfigStorage_t;

#define CONFIG_STORAGE_DEFAULTS { \
  .flags = NC_CFG_FLAG_ENABLE_TILT_INDICATOR, \
}

bool nc_config_get(eNabuCasaConfigKey key);
void nc_config_set(eNabuCasaConfigKey key, bool value);

#endif /* APPS_SERIALAPI_CMD_PROPRIETARY_H_ */
