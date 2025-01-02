
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
} eNabuCasaCmd;

#endif /* APPS_SERIALAPI_CMD_PROPRIETARY_H_ */
