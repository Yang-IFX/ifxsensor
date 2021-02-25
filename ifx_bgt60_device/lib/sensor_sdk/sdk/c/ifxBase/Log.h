/**
* \copyright
* MIT License
*
* Copyright (c) 2021 Infineon Technologies AG
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file Log.h
*
* \brief   \copybrief gr_log
*
* For details refer to \ref gr_log
*
*
* @{
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

/**
* @}
*/