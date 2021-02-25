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
* SOFTWARE..
*
* \endcopyright
*
* \author Infineon Technologies AG
*
*
* @{
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

/**
* @}
*/