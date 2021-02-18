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
* \file Target.h
*
* \brief   Definitions used by Tracker algorithm.
*
*
* @{
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

/**
* @}
*/