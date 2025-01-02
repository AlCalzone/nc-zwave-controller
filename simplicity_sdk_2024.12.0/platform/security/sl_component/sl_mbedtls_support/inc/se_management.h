/***************************************************************************//**
 * @file
 * @brief Silicon Labs SE device management interface.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef SE_MANAGEMENT_H
#define SE_MANAGEMENT_H

/// @cond DO_NOT_INCLUDE_WITH_DOXYGEN

/***************************************************************************//**
 * \addtogroup sl_mbedtls_plugins
 * \{
 ******************************************************************************/

/***************************************************************************//**
 * \addtogroup sl_se_management Peripheral Instance Management: Secure Engine
 * \brief Concurrency management functions for Secure Engine mailbox access
 *
 * \{
 ******************************************************************************/

#include "em_device.h"

#if defined(SEMAILBOX_PRESENT)

#include "sli_se_manager_mailbox.h"
#include "sli_se_manager_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          Get ownership of the SE mailbox
 *
 * \return         0 if successful, negative on error
 */
__STATIC_INLINE int se_management_acquire(void)
{
  // Acquire SE manager lock
  return sli_se_lock_acquire() == SL_STATUS_OK ? 0 : -1;
}

/**
 * \brief          Release ownership of the SE mailbox
 *
 * \return         0 if successful, negative on error
 */
__STATIC_INLINE int se_management_release(void)
{
  // Release SE manager lock
  return sli_se_lock_release() == SL_STATUS_OK ? 0 : -1;
}

/**
 * \brief          Handle the response of the previously executed command.
 *
 * \details        This function handles the response of the previously
 *                 executed HSE command by calling sli_se_mailbox_read_response
 *                 to read the response value and returns it. For Series-3 this
 *                 function executes sli_se_mailbox_read_response inside an
 *                 atomic section and clears the SEMAILBOX FIFO at the end.
 *
 * \note           This function implements a workaround that is planned to be
 *                 replaced in https://jira.silabs.com/browse/PSEC-5643.
 *
 * \return         Value returned by sli_se_mailbox_read_response.
 ******************************************************************************/
__STATIC_INLINE sli_se_mailbox_response_t sli_se_handle_mailbox_response(void)
{
  #if defined(_SILICON_LABS_32B_SERIES_3)
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
  #endif

  // Read command response
  sli_se_mailbox_response_t se_mailbox_response = sli_se_mailbox_read_response();

  #if defined(_SILICON_LABS_32B_SERIES_3)
  CORE_EXIT_ATOMIC();

  // Read the command handle word ( not used ) from the SEMAILBOX FIFO
  SEMAILBOX_HOST->FIFO;
  #endif

  // Return command response
  return se_mailbox_response;
}

#ifdef __cplusplus
}
#endif

#endif /* SEMAILBOX_PRESENT */

/** \} (end addtogroup sl_se_management) */
/** \} (end addtogroup sl_mbedtls_plugins) */

/// @endcond

#endif /* SE_MANAGEMENT_H */
