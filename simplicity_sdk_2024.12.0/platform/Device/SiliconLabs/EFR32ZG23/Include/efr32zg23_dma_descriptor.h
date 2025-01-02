/**************************************************************************//**
 * @file
 * @brief EFR32ZG23 DMA descriptor bit field definitions
 ******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories, Inc. www.silabs.com</b>
 ******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is Third Party Software licensed by Silicon Labs from a third party
 * and is governed by the sections of the MSLA applicable to Third Party
 * Software and the additional terms set forth below.
 *
 *****************************************************************************/
#ifndef EFR32ZG23_DMA_DESCRIPTOR_H
#define EFR32ZG23_DMA_DESCRIPTOR_H

#if defined(__ICCARM__)
#pragma system_include       /* Treat file as system include file. */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang system_header  /* Treat file as system include file. */
#endif

/**************************************************************************//**
* @addtogroup Parts
* @{
******************************************************************************/
/**************************************************************************//**
 * @defgroup DMA_DESCRIPTOR DMA Descriptor
 * @{
 *****************************************************************************/
/** DMA_DESCRIPTOR Register Declaration */
typedef struct {
  /* Note! Use of double __IOM (volatile) qualifier to ensure that both */
  /* pointer and referenced memory are declared volatile. */
  __IOM uint32_t     CTRL;     /**< DMA control register */
  __IOM void * __IOM SRC;      /**< DMA source address */
  __IOM void * __IOM DST;      /**< DMA destination address */
  __IOM void * __IOM LINK;     /**< DMA link address */
} DMA_DESCRIPTOR_TypeDef;      /**< @} */

/** @} End of group Parts */

#endif // EFR32ZG23_DMA_DESCRIPTOR_H
