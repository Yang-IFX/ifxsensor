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

#ifndef IFX_RADAR_UTIL_H
#define IFX_RADAR_UTIL_H

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
#include <stdbool.h>

#include "ifxBase/Forward.h"

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

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/**
 * @brief Convert string to uuid
 *
 * Convert string to a uint8_t uuid buffer.
 *
 * @param [in]  string  string representation of uuid
 * @param [out] uuid    array representation of uuid
 * @retval true         if conversion was successful
 * @retval false        if the string is not a valid uuid
 */
IFX_DLL_PUBLIC
bool ifx_util_string_to_uuid(const char* string, uint8_t uuid[16]);

/**
 * @brief Convert uuid to string
 *
 * Convert the uint8_t uuid buffer to the canonical string representation. The
 * uuid is written to string. The pointer string must have enough space for at
 * least 37 characters (36 characters of the uuid, and a terminating null
 * character).
 *
 * @param [in]  uuid    array representation of uuid
 * @param [out] string  string representation of uuid
 */
IFX_DLL_PUBLIC
void ifx_util_uuid_to_string(const uint8_t uuid[16], char* string);

 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_UTIL_H */
