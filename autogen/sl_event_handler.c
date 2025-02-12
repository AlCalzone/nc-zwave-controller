#include "sl_event_handler.h"

#include "em_chip.h"
#include "sl_interrupt_manager.h"
#include "sl_device_init_dcdc.h"
#include "sl_clock_manager.h"
#include "sl_hfxo_manager.h"
#include "sl_device_init_hfxo.h"
#include "sl_device_init_clocks.h"
#include "sl_memory_manager.h"
#include "pa_conversions_efr32.h"
#include "sl_rail_util_power_manager_init.h"
#include "btl_interface.h"
#include "sl_sleeptimer.h"
#include "sl_mpu.h"
#include "sl_gpio.h"
#include "gpiointerrupt.h"
#include "sl_mbedtls.h"
#include "sl_simple_button_instances.h"
#include "ZW_basis_api.h"
#include "psa/crypto.h"
#include "sl_se_manager.h"
#include "cmsis_os2.h"
#include "nvm3_default.h"
#include "sl_power_manager.h"

void sl_platform_init(void)
{
  CHIP_Init();
  sl_interrupt_manager_init();
  sl_device_init_dcdc();
  sl_clock_manager_runtime_init();
  sl_hfxo_manager_init_hardware();
  sl_device_init_hfxo();
  sl_device_init_clocks();
  sl_memory_init();
  bootloader_init();
  osKernelInitialize();
  sl_zwave_platform_startup();
  nvm3_initDefault();
  sl_power_manager_init();
}

void sl_kernel_start(void)
{
  osKernelStart();
}

void sl_driver_init(void)
{
  sl_gpio_init();
  GPIOINT_Init();
  sl_simple_button_init_instances();
}

void sl_service_init(void)
{
  sl_sleeptimer_init();
  sl_hfxo_manager_init();
  sl_mpu_disable_execute_from_ram();
  sl_mbedtls_init();
  psa_crypto_init();
  sl_se_init();
}

void sl_stack_init(void)
{
  sl_rail_util_pa_init();
  sl_rail_util_power_manager_init();
  sl_zwave_protocol_startup();
}

void sl_internal_app_init(void)
{
}

