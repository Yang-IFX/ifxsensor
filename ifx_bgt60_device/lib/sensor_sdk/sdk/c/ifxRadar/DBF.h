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
* \file DBF.h
*
* \brief   \copybrief gr_dbf
*
* For details refer to \ref gr_dbf
*
*
* @{
*/


#ifndef IFX_RADAR_DBF_H
#define IFX_RADAR_DBF_H

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
 * @brief A handle for an instance of DigitalBeamForming module, see DBF.h
 */
typedef struct ifx_DBF_s* ifx_DBF_Handle_t;

/**
 * @brief Defines the structure for DBF module related settings.
 */
typedef struct
{
    uint8_t     num_beams;    /**< Number of beams.*/
    uint8_t     num_antennas; /**< Number of antennas.*/
    ifx_Float_t min_angle;    /**< Minimum angle on left side of FoV.*/
    ifx_Float_t max_angle;    /**< Maximum angle on right side of FoV.*/
    ifx_Float_t d_by_lambda;  /**< Ratio between antenna spacing 'd' and wavelength.*/
} ifx_DBF_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_dbf DBF
  * @brief API for Digital Beamforming (DBF) algorithm
  *
  * Digital Beamforming (DBF) is a method to focus a radar transmitter or receiver
  * in a certain direction in 3D space. In this context, the left to right direction
  * is commonly referred to as azimuth and the high to low direction as elevation.
  * The antennas of DBF systems are set up as a Uniform Linear Array (ULA) while
  * each direction needs a ULA of at least two antennas positioned in its plane to
  * perform beamforming. Thus, the minimum requirement for detecting azimuth and
  * elevation is a ULA with three antennas in an L shape.
  * In DBF all ULA's antenna signals are sampled and therefore present for processing
  * at the same time. Thus DBF allows for a precise and simultaneous processing of
  * different angles and frequencies, only limited by the computational effort, but
  * theoretically with arbitrary precision. The theory and implementation of DBF
  * algorithms as described here can not only be used to analyze and weight signals
  * in 3D space but also for FFT processed spectral representations or derived
  * properties like range doppler maps.
  *
  * Input to this module is a cube i.e. an array of range doppler spectrums
  * (complex values) over all rx antennas and ouput is also a complex cube
  * i.e. stack of matrices (of dimension equal to the dimension of range doppler
  * spectrum) and number of slices of cube are equal to the number of beams.
  *
  * An algorithm explanation is also available at the \ref ssct_radarsdk_algorithms_dbf SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates a DBF handle (object), based on the input parameters
 *
 * @param [in]     config    DBF configurations defined by \ref ifx_DBF_Config_t.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_DBF_Handle_t ifx_dbf_create(ifx_DBF_Config_t* config);

/**
 * @brief Computes beams for a given range Doppler spectrum overs across Rx antennas.
 *
 * @param [in]     handle              A handle to the DBF object
 * @param [in]     rng_dopp_spectrum   A complex Cube (3D) of range Doppler spectrum for all Rx channels i.e.
 *                                     (Nsamples x NumChirps x Number of Antennas)
 * @param [out]    rng_dopp_image_beam A complex Cube (3D) containing range Doppler image beams i.e.
 *                                     (Nsamples x NumChirps x NumberofBeams)
 *
 */
IFX_DLL_PUBLIC
void ifx_dbf_run_c(ifx_DBF_Handle_t handle,
                   ifx_Cube_C_t* rng_dopp_spectrum,
                   ifx_Cube_C_t* rng_dopp_image_beam);

/**
 * @brief Performs destruction of DBF handle (object) to clear internal states and memories.
 *
 * @param [in]     handle    A handle to the DBF object
 *
 */
IFX_DLL_PUBLIC
void ifx_dbf_destroy(ifx_DBF_Handle_t handle);

/**
 * @brief Returns number of beams configured in the DBF handle
 *
 * @param [in]     handle    A handle to the DBF object
 *
 * @return  Currently configured number of beams
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_dbf_get_beam_count(ifx_DBF_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_DBF_H */

/**
* @}
*/