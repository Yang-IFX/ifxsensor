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
 * @file Target.h
 *
 * @brief Definitions used by Tracker algorithm.
 */

#ifndef IFX_RADAR_TARGET_H
#define IFX_RADAR_TARGET_H

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

#include "ifxBase/Forward.h"

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

#define IFX_OFFSET_OF(type, member)          offsetof(type, member)
#define IFX_TARGET_COL_OF(member)            (IFX_OFFSET_OF(ifx_Target_t, member) / sizeof(ifx_Float_t))

#define IFX_TARGET_RANGE_COL_OFFSET          IFX_TARGET_COL_OF(range)  
#define IFX_TARGET_RANGE(m, r)               IFX_MAT_AT(m, r, 0)
#define IFX_TARGET_SPEED_COL_OFFSET          IFX_TARGET_COL_OF(speed)  
#define IFX_TARGET_SPEED(m, r)               IFX_MAT_AT(m, r, IFX_TARGET_SPEED_COL_OFFSET)
#define IFX_TARGET_AZIMUTH_ANGLE_COL_OFFSET  IFX_TARGET_COL_OF(azimuth_angle)  
#define IFX_TARGET_AZIMUTH_ANGLE(m, r)       IFX_MAT_AT(m, r, IFX_TARGET_AZIMUTH_ANGLE_COL_OFFSET)
#define IFX_TARGET_COORDS_COL_OFFSET         IFX_TARGET_COL_OF(coords) 
#define IFX_TARGET_COORDS_LEN                3
#define IFX_TARGET_COORD(m, r, c)            IFX_MAT_AT(m, r, c + IFX_TARGET_COORDS_COL_OFFSET)
#define IFX_TARGET_MATRIX_CREATE(r)          ifx_mat_create_r(r, 6);

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @brief Defines the structure for Target module.
 */
typedef struct
{
    ifx_Float_t range;                          /**< Range of the target.*/
    ifx_Float_t speed;                          /**< Speed of the target.*/
    ifx_Float_t azimuth_angle;                  /**< Azimuth angle of the target.*/
    ifx_Float_t coords[IFX_TARGET_COORDS_LEN];  /**< Cartesian coordinates of the target detections.*/
} ifx_Target_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_TARGET_H */
