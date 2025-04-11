#include <app.h>
#include <cmds_proprietary.h>
#include <string.h>
#include "serialapi_file.h"

#define BYTE_INDEX(x) (x / 8)
#define BYTE_OFFSET(x) (1 << (x % 8))
#define BITMASK_ADD_CMD(bitmask, cmd) (bitmask[BYTE_INDEX(cmd)] |= BYTE_OFFSET(cmd))

#define SUPPORT_LED 1
#define SUPPORT_GYRO 1

#if SUPPORT_LED
#include "drivers/ws2812.h"
extern LedEffect_t ledEffect;
extern LedMode_t ledMode;
#endif
#if SUPPORT_GYRO
#include "drivers/qma6100p.h"
extern bool bRequestGyroMeasurement;
#endif

void func_id_nabu_casa(uint8_t inputLength,
                       const uint8_t *pInputBuffer,
                       uint8_t *pOutputBuffer,
                       uint8_t *pOutputLength)
{
  uint8_t i = 0;
  uint8_t cmdRes;

  /* We assume operation is non-successful */
  cmdRes = false;

  if (1 > inputLength)
  {
    /* Command length must be at least 1 byte. Return with negative response in the out buffer */
    pOutputBuffer[i++] = cmdRes;
    *pOutputLength = i;
    return;
  }

  pOutputBuffer[i++] = pInputBuffer[0]; /* Set output command ID equal input command ID */
  switch (pInputBuffer[0])
  {

  /* Report which subcommands are supported beside the NABU_CASA_CMD_SUPPORTED */
  case NABU_CASA_CMD_SUPPORTED:
    // HOST->ZW: NABU_CASA_CMD_SUPPORTED
    // ZW->HOST: NABU_CASA_CMD_SUPPORTED | supportedBitmask

    /* Report all supported commands as bitmask of their values */
    uint8_t supportedBitmask[32];
    memset(supportedBitmask, 0, sizeof(supportedBitmask));
    // Mark each command as supported
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_CMD_SUPPORTED);
#if SUPPORT_LED
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_LED_GET);
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_LED_SET);
#endif
#if SUPPORT_GYRO
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_GYRO_MEASURE);
#endif

    // Copy as few bytes as necessary into the output buffer
    for (int j = 0; j <= NABU_CASA_CMD_SUPPORTED / 8; j++)
    {
      pOutputBuffer[i++] = supportedBitmask[j];
    }
    break;

#if SUPPORT_LED
  case NABU_CASA_LED_GET:
    // HOST->ZW: NABU_CASA_LED_GET
    // ZW->HOST: NABU_CASA_LED_GET | r | g | b |

    // Get the current state of the LED
    rgb_t led_color = get_color_buffer();

    pOutputBuffer[i++] = led_color.R;
    pOutputBuffer[i++] = led_color.G;
    pOutputBuffer[i++] = led_color.B;
    break;

  case NABU_CASA_LED_SET:
    // HOST->ZW: NABU_CASA_LED_SET | r | g | b | [ effect | speed ]
    // ZW->HOST: NABU_CASA_LED_SET | true

    ledMode = LED_MODE_MANUAL;
    /*if (ledMode != LED_MODE_MANUAL)
    {
      cmdRes = false;
    }
    else*/ if (inputLength >= 4)
    {
      uint8_t r = pInputBuffer[1];
      uint8_t g = pInputBuffer[2];
      uint8_t b = pInputBuffer[3];
      rgb_t color = {g, r, b};

      if (inputLength >= 5 && pInputBuffer[4] != NC_LED_FX_SOLID)
      {
        // Parse effect
        // Default duration is 1s
        uint8_t ticksPerStep = 1;
        if (inputLength >= 6)
        {
          // Parse speed
          ticksPerStep = pInputBuffer[5];
        }
        // Set fade effect as the LED effect
        LedEffectFade_t fade = {
            .color = color,
            .brightness = 0xff,
            .increasing = false,
            .ticksPerStep = ticksPerStep,
            .tickCounter = 0};
        ledEffect = (LedEffect_t){
            .type = LED_EFFECT_FADE,
            .effect.fade = fade};
      }
      else
      {
        // Set solid color as the LED effect
        LedEffectSolid_t solid = {
            .color = color,
            .modified = true};
        ledEffect = (LedEffect_t){
            .type = LED_EFFECT_SOLID,
            .effect.solid = solid};
      }

      // Store the current color in NVM, so it can be restored after a reboot
      NabuCasaLedStorage_t ledStorage = {
          .valid = true,
          .r = r,
          .g = g,
          .b = b};
      SerialApiNvmWriteAppData(NC_APPDATA_OFFSET_LED, (uint8_t *)&ledStorage, sizeof(ledStorage));

      cmdRes = true;
    }
    pOutputBuffer[i++] = cmdRes;
    break;
#endif

#if SUPPORT_GYRO
  case NABU_CASA_GYRO_MEASURE:
    // HOST->ZW (REQ): NABU_CASA_GYRO_MEASURE
    // ZW->HOST (RES): NABU_CASA_GYRO_MEASURE | true
    // later
    // ZW->HOST (CB):  NABU_CASA_GYRO_MEASURE
    //           | accel_x (MSB) | accel_x (LSB)
    //           | accel_y (MSB) | accel_y (LSB)
    //           | accel_z (MSB) | accel_z (LSB)

    bRequestGyroMeasurement = true;
    cmdRes = true;
    pOutputBuffer[i++] = cmdRes;
    break;
#endif

  default:
    // Unsupported. Return false
    pOutputBuffer[i++] = false;
    break;
  }

  *pOutputLength = i;
}
