/* ===========================================================================
** Copyright (C) 2019-2020 Infineon Technologies AG. All rights reserved.
** ===========================================================================
**
** ===========================================================================
** Infineon Technologies AG (INFINEON) is supplying this file for use
** exclusively with Infineon's sensor products. This file can be freely
** distributed within development tools and software supporting such
** products.
**
** THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
** OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
** INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT,
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON
** WHATSOEVER.
** ===========================================================================
*/
/**
 * @file Log.h
 *
 * \brief \copybrief gr_log
 *
 * For details refer to \ref gr_log
 */

#ifndef IFX_BASE_LOG_H
#define IFX_BASE_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#ifndef IFX_STDOUT
#include <stdio.h>
#define IFX_STDOUT stderr
#endif

#include "ifxBase/Forward.h"

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

#if defined(IFX_LOG_SEVERITY_DEBUG)
#define IFX_LOG_DEBUG(...) ifx_log(IFX_STDOUT, IFX_LOG_DEBUG, __VA_ARGS__)
#else
#define IFX_LOG_DEBUG(...)
#endif

#if defined(IFX_LOG_SEVERITY_INFO) || \
    defined(IFX_LOG_SEVERITY_DEBUG)
#define IFX_LOG_INFO(...) ifx_log(IFX_STDOUT, IFX_LOG_INFO, __VA_ARGS__)
#else
#define IFX_LOG_INFO(...)
#endif

#if defined(IFX_LOG_SEVERITY_INFO)    || \
    defined(IFX_LOG_SEVERITY_WARNING) || \
    defined(IFX_LOG_SEVERITY_DEBUG)
#define IFX_LOG_WARNING(...) ifx_log(IFX_STDOUT, IFX_LOG_WARNING, __VA_ARGS__)
#else
#define IFX_LOG_WARNING(...)
#endif

#if defined(IFX_LOG_SEVERITY_INFO)    || \
    defined(IFX_LOG_SEVERITY_WARNING) || \
    defined(IFX_LOG_SEVERITY_ERROR)   || \
    defined(IFX_LOG_SEVERITY_DEBUG)
#define IFX_LOG_ERROR(...)   ifx_log(IFX_STDOUT, IFX_LOG_ERROR, __VA_ARGS__)
#else
#define IFX_LOG_ERROR(...)
#endif

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @brief Defines supported Log options.
 */
typedef enum
{
    IFX_LOG_INFO,
    IFX_LOG_WARNING,
    IFX_LOG_ERROR,
    IFX_LOG_DEBUG
} ifx_Log_Severity_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_SDK_base
  * @{
  */

/** @defgroup gr_log Log
  * @brief API for logging
  * @{
  */

IFX_DLL_PUBLIC
void ifx_log(FILE* f, ifx_Log_Severity_t s, char* msg, ...);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_LOG_H */
