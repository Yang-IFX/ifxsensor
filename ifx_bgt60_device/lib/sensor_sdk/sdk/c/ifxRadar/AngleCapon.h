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
* \file AngleCapon.h
*
* \brief   \copybrief gr_anglecapon
*
* For details refer to \ref gr_anglecapon
*
*
* @{
*/

#ifndef IFX_RADAR_ANGLE_CAPON_H
#define IFX_RADAR_ANGLE_CAPON_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

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

/**
 * @brief A handle for an instance of AngleCapon module, see AngleCapon.h
 */
typedef struct ifx_AngleCapon_s* ifx_AngleCapon_Handle_t;

/**
 * @brief Defines the structure for Angle Capon module related settings.
 */
typedef struct
{
    uint8_t     range_win_size;       /**< Range window size.*/
    uint8_t     selected_rx;          /**< Select the best Rx channel for choosing proper Doppler index.*/
    uint16_t    chirps_per_frame;     /**< Number of chirps per frame.*/
    ifx_Float_t phase_offset_degrees; /**< Phase offset compensation between used Rx antennas in degrees.*/
    uint8_t     num_virtual_antennas; /**< Virtual number of antennas.*/
    uint8_t     num_beams;            /**< Number of beams.*/
    ifx_Float_t min_angle_degrees;    /**< Minimum angle on left side of FoV in degrees.*/
    ifx_Float_t max_angle_degrees;    /**< Maximum angle on right side of FoV in degrees.*/
    ifx_Float_t d_by_lambda;          /**< Ratio between antenna spacing 'd' and wavelength.*/
} ifx_AngleCapon_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_anglecapon Angle Capon
  * @brief API for Angle Capon algorithm
  *
  * This algorithm is used to generate the angle estimation of target
  * at specific range gate and maximum Doppler associated with it.
  *
  * An algorithm explanation is available at the \ref ssct_radarsdk_algorithms_detect_capon SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates a AngleCapon handle (object), based on the input parameters.
 *
 * @param [in]     config              AngleCapon configurations defined by \ref ifx_AngleCapon_Config_t.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_AngleCapon_Handle_t ifx_anglecapon_create(ifx_AngleCapon_Config_t* config);

/**
 * @brief Runs angle capon algorithm, based on the input parameters.
 *
 * @param [in]     handle              A handle to the AngleCapon object
 * @param [in]     range_idx           ...
 * @param [in]     rx_spectrum         Range Doppler spectrum for all RX channels (Nsamples x NChirps x NAntennas)
 * @param [out]    angle_idx           Angle index that indicate target among number of beams.
 *
 * @return Angle value.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_anglecapon_run(const ifx_AngleCapon_Handle_t handle,
                               const uint32_t range_idx,
                               ifx_Cube_C_t* rx_spectrum,
                               uint32_t* angle_idx);

/**
 * @brief Destroys AngleCapon handle (object) to clear internal states and memories.
 *
 * @param [in]     handle              A handle to the AngleCapon object
 *
 */
IFX_DLL_PUBLIC
void ifx_anglecapon_destroy(ifx_AngleCapon_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_ANGLE_CAPON_H */

/**
* @}
*/