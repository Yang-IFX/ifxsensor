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
 * @file time_formatter.h
 *
 * @brief This file defines the API for time formatting.
 *
 */

#ifndef TIME_FORMATTER_H
#define TIME_FORMATTER_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdint.h>
#include "ifxBase/Types.h"
#include "ifxBase/Error.h"

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/*
==============================================================================
   3. TYPES
==============================================================================
*/

typedef struct ifx_Time_s* ifx_Time_Handle_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

ifx_Error_t ifx_time_create(ifx_Time_Handle_t* handle);

int64_t ifx_time_get_ms(ifx_Time_Handle_t handle);

char* ifx_time_get_cstr(ifx_Time_Handle_t handle);

void ifx_time_destroy(ifx_Time_Handle_t handle);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* TIME_FORMATTER_H */
