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
 * @file Forward.h
 *
 * @brief Forward declaration of structures
 *
 * Defines forward declaration for all publicly visible structures
 * and their respective type definitions.
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
