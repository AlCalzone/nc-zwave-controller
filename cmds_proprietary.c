#include <app.h>
#include <cmds_proprietary.h>
#include <string.h>
#include <ZAF_nvm_app.h>

#define BYTE_INDEX(x) (x / 8)
#define BYTE_OFFSET(x) (1 << (x % 8))
#define BITMASK_ADD_CMD(bitmask, cmd) (bitmask[BYTE_INDEX(cmd)] |= BYTE_OFFSET(cmd))

#define SUPPORT_LED 1
#define SUPPORT_GYRO 1

#if SUPPORT_LED
#include "drivers/ws2812.h"
extern LedEffect_t ledEffectUser;
extern LedEffect_t ledEffectSystem;
extern LedEffect_t ledEffectDefault;
extern void trigger_led_effect_refresh(void);
#endif
#if SUPPORT_GYRO
#include "drivers/qma6100p.h"
extern bool bRequestGyroMeasurement;
extern bool bEnableTiltDetection;
#endif

bool nc_config_get(eNabuCasaConfigKey key)
{
  NabuCasaConfigStorage_t cfg = CONFIG_STORAGE_DEFAULTS;
  ZAF_nvm_app_read(FILE_ID_NABUCASA_CONFIG, &cfg, sizeof(cfg));
  return (cfg.flags & (1 << key)) != 0;
}

void nc_config_set(eNabuCasaConfigKey key, bool value)
{
  NabuCasaConfigStorage_t cfg = CONFIG_STORAGE_DEFAULTS;
  ZAF_nvm_app_read(FILE_ID_NABUCASA_CONFIG, &cfg, sizeof(cfg));

  if (value)
  {
    cfg.flags |= (1 << key);
  }
  else
  {
    cfg.flags &= ~(1 << key);
  }

  ZAF_nvm_app_write(FILE_ID_NABUCASA_CONFIG, &cfg, sizeof(cfg));
}

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
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_SYSTEM_INDICATION_SET);
#endif
#if SUPPORT_GYRO
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_GYRO_MEASURE);
#endif
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_CONFIG_GET);
    BITMASK_ADD_CMD(supportedBitmask, NABU_CASA_CONFIG_SET);

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

    if (inputLength >= 4)
    {
      int pos = 1;
      uint8_t r = pInputBuffer[pos++];
      uint8_t g = pInputBuffer[pos++];
      uint8_t b = pInputBuffer[pos++];
      rgb_t color = {g, r, b};

      switch (pInputBuffer[pos++])
      {
      case NC_LED_FX_NOT_SET:
      {
        // Clear the LED effect
        ledEffectUser = (LedEffect_t){
            .type = LED_EFFECT_NOT_SET};
        break;
      }

      case NC_LED_FX_SOLID:
      {
        // Set solid color as the LED effect
        LedEffectSolid_t solid = {
            .color = color,
            .modified = true};
        ledEffectUser = (LedEffect_t){
            .type = LED_EFFECT_SOLID,
            .effect.solid = solid};
        break;
      }

      case NC_LED_FX_FADE:
      {
        // Parse effect
        // Default duration is 1s
        uint8_t ticksPerStep = 1;
        if (inputLength > pos)
        {
          // Parse speed
          ticksPerStep = pInputBuffer[pos++];
        }

        // Set fade effect as the LED effect
        LedEffectFade_t fade = {
            .color = color,
            .rawColor = false,
            .brightness = FADE_MAX_BRIGHTNESS,
            .increasing = false,
            .ticksPerStep = ticksPerStep,
            .stepSize = 1,
            .tickCounter = 0};
        ledEffectUser = (LedEffect_t){
            .type = LED_EFFECT_FADE,
            .effect.fade = fade};
        break;
      }
      }

      // Store the current color in NVM, so it can be restored after a reboot
      NabuCasaLedStorage_t ledStorage;
      // FIXME: Only override if something changed

      if (ledEffectUser.type == LED_EFFECT_NOT_SET)
      {
        ledStorage = (NabuCasaLedStorage_t){
            .valid = false,
            .r = 0,
            .g = 0,
            .b = 0};
      }
      else
      {
        ledStorage = (NabuCasaLedStorage_t){
            .valid = true,
            .r = r,
            .g = g,
            .b = b};
      }
      ZAF_nvm_app_write(FILE_ID_NABUCASA_LED, &ledStorage, sizeof(ledStorage));

      cmdRes = true;
    }
    pOutputBuffer[i++] = cmdRes;
    break;

  case NABU_CASA_SYSTEM_INDICATION_SET:
    // HOST->ZW (REQ): NABU_CASA_SYSTEM_INDICATION_SET | severity
    // ZW->HOST (RES): NABU_CASA_SYSTEM_INDICATION_SET | true

    if (inputLength >= 2)
    {
      eNabuCasaSystemIndication severity = (eNabuCasaSystemIndication)pInputBuffer[1];
      switch (severity)
      {
      case NC_SYS_INDICATION_OFF:
        ledEffectSystem = (LedEffect_t){
            .type = LED_EFFECT_NOT_SET};
        // Force-swap the current effect
        trigger_led_effect_refresh();

        cmdRes = true;
        break;
      case NC_SYS_INDICATION_WARN:
        ledEffectSystem = (LedEffect_t){
            .type = LED_EFFECT_SOLID,
            .effect.solid = {
                .color = yellow,
                .modified = true}};
        cmdRes = true;
        break;
      case NC_SYS_INDICATION_ERROR:
        ledEffectSystem = (LedEffect_t){
            .type = LED_EFFECT_SOLID,
            .effect.solid = {
                .color = red,
                .modified = true}};
        cmdRes = true;
        break;
      default:
        // Unsupported. Do nothing
        break;
      }
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

  case NABU_CASA_CONFIG_GET:
    // HOST->ZW (REQ): NABU_CASA_CONFIG_GET | key
    // ZW->HOST (RES): NABU_CASA_CONFIG_GET | key | size | value

    if (inputLength >= 2)
    {
      eNabuCasaConfigKey key = (eNabuCasaConfigKey)pInputBuffer[1];
      bool value = nc_config_get(key);
      pOutputBuffer[i++] = key;
      pOutputBuffer[i++] = 1;
      pOutputBuffer[i++] = value;
    }
    else
    {
      // invalid command
      pOutputBuffer[i++] = 0xFF;
    }
    break;

  case NABU_CASA_CONFIG_SET:
    // HOST->ZW (REQ): NABU_CASA_CONFIG_SET | key | size | value
    // ZW->HOST (RES): NABU_CASA_CONFIG_SET | success

    if (inputLength >= 4)
    {
      eNabuCasaConfigKey key = (eNabuCasaConfigKey)pInputBuffer[1];
      uint8_t size = pInputBuffer[2];
      if (size > 0 && size <= 4 && inputLength >= 3 + size)
      {
        int32_t value = 0;
        for (int j = 0; j < size; j++)
        {
          value |= (pInputBuffer[3 + j] << (j * 8));
        }

        switch (key)
        {
        case NC_CFG_ENABLE_TILT_INDICATOR:
        {
          // Save change in NVM
          bool enable = (value != 0);
          nc_config_set(key, enable);
          // and forward to application
          bEnableTiltDetection = enable;
          trigger_led_effect_refresh();

          cmdRes = true;
          break;
        }
        default:
          // Unsupported. Do nothing
          break;
        }
      }
    }

    pOutputBuffer[i++] = cmdRes;
    break;

  default:
    // Unsupported. Return false
    pOutputBuffer[i++] = false;
    break;
  }

  *pOutputLength = i;
}
