#include <app.h>
#include <cmds_proprietary.h>
#include <string.h>

#define BYTE_INDEX(x) (x / 8)
#define BYTE_OFFSET(x) (1 << (x % 8))
#define BITMASK_ADD_CMD(bitmask, cmd) (bitmask[BYTE_INDEX(cmd)] |= BYTE_OFFSET(cmd))

#define SUPPORT_LED 1
#define SUPPORT_GYRO 1

#if SUPPORT_LED
#include "drivers/ws2812.h"
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

  /* We assume operation is nonesuccessful */
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
    // ZW->HOST: NABU_CASA_LED_GET | num_leds | r_0 | g_0 | b_0 | ... | r_n | g_n | b_n |

    // Get the current state of the LED
    rgb_t led_colors[NUMBER_OF_LEDS] = {};
    get_color_buffer(led_colors);

    pOutputBuffer[i++] = NUMBER_OF_LEDS;
    for (int j = 0; j < NUMBER_OF_LEDS; j++)
    {
      pOutputBuffer[i++] = led_colors[j].R;
      pOutputBuffer[i++] = led_colors[j].G;
      pOutputBuffer[i++] = led_colors[j].B;
    }
    break;

  case NABU_CASA_LED_SET:
    // HOST->ZW: NABU_CASA_LED_SET | num_entries | idx_0 | r | g | b | ... | idx_n | r | g | b |
    // ZW->HOST: NABU_CASA_LED_SET | true

    if (inputLength >= 1)
    {
      uint8_t num_entries = pInputBuffer[1];
      if (num_entries <= NUMBER_OF_LEDS && inputLength >= 1 + 4 * num_entries) {
        // Get the current state of the LED
        rgb_t led_colors[NUMBER_OF_LEDS] = {};
        get_color_buffer(led_colors);
        // Incrementally modify it
        for (int e = 0; e < num_entries; e++) {
          int offset = 2 + 4 * e;
          int idx = pInputBuffer[offset];
          if (idx < NUMBER_OF_LEDS) {
            led_colors[idx].R = pInputBuffer[offset + 1];
            led_colors[idx].G = pInputBuffer[offset + 2];
            led_colors[idx].B = pInputBuffer[offset + 3];
          }
        }

        // Set the new state of the LED
        set_color_buffer(led_colors);

        cmdRes = true;
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

  default:
    // Unsupported. Return false
    pOutputBuffer[i++] = false;
    break;
  }

  *pOutputLength = i;
}