/**
* \copyright
* MIT License
*
* Copyright (c) 2020 Infineon Technologies AG
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
* SOFTWARE
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file Forward.h
*
* \brief   Forward declaration of structures
*
* Defines forward declaration for all publicly visible structures
* and their respective type definitions.
*
*
* @{
*/


#ifndef IFX_BASE_FORWARD_H
#define IFX_BASE_FORWARD_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stddef.h>
#include "ifxBase/Types.h"

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

/**
 * @brief Forward declaration structure to operate on Complex.
 */
typedef struct ifx_Complex_s ifx_Complex_t;

/**
 * @brief Forward declaration structure to operate on real Cube.
 */
typedef struct ifx_Cube_R_s ifx_Cube_R_t;

/**
 * @brief Forward declaration structure to operate on complex Cube.
 */
typedef struct ifx_Cube_C_s ifx_Cube_C_t;

/**
 * @brief Forward declaration structure for Math Axis Spec.
 */
typedef struct ifx_Math_Axis_Spec_s ifx_Math_Axis_Spec_t;

/**
 * @brief Forward declaration structure for real Matrix.
 */
typedef struct ifx_Matrix_R_s ifx_Matrix_R_t;

/**
 * @brief Forward declaration structure for complex Matrix.
 */
typedef struct ifx_Matrix_C_s ifx_Matrix_C_t;

/**
 * @brief Forward declaration structure for Polar type.
 */
typedef struct ifx_Polar_s ifx_Polar_t;

/**
 * @brief Forward declaration structure to operate on Real Vector.
 */
typedef struct ifx_Vector_R_s ifx_Vector_R_t;

/**
 * @brief Forward declaration structure to operate on Complex Vector.
 */
typedef struct ifx_Vector_C_s ifx_Vector_C_t;


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_FORWARD_H */

/**
* @}
*/