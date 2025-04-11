/***************************************************************************//**
 * @file main.c
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "sl_system_init.h"
#include "sl_system_kernel.h"
#include "stdint.h"

int main(void)
{
  // Fix incorrect max. TX Power on Simplicity SDK
  // Equivalent to m_TxPowerMode = ZW_RADIO_TX_POWER_MODE_20DBM;
  uint8_t* m_TxPowerMode = (uint8_t*)0x2000950e;
  *m_TxPowerMode = 1;

  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that processing task(s) will be created by this call.
  sl_system_init();

  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
}
