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
* \file RangeAngleImage.h
*
* \brief   \copybrief gr_rai
*
* For details refer to \ref gr_rai
*
*
* @{
*/


#ifndef IFX_RADAR_RANGE_ANGLE_IMAGE_H
#define IFX_RADAR_RANGE_ANGLE_IMAGE_H

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

#include "ifxRadar/DBF.h"
#include "ifxRadar/RangeDopplerMap.h"


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
 * @brief A handle for an instance of Range Angle Image module, see RangeAngleImage.h.
 */
typedef struct ifx_RAI_s* ifx_RAI_Handle_t;

/**
 * @brief Defines the structure for Range Angle Image module related settings.
 */
typedef struct
{
    ifx_RDM_Config_t  rdm_config;         /**< Range doppler map configurations.*/
    ifx_Float_t       alpha_mti_filter;   /**< 2D MTI filter coefficient.*/
    ifx_DBF_Config_t  dbf_config;         /**< Digital beamforming module configurations.*/
    uint32_t          num_of_images;      /**< Number of images (responses) for Range Angle Image.*/
    uint32_t          num_antenna_array;  /**< Number of virtual antennas.*/
} ifx_RAI_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */
 
/** @defgroup gr_rai Range Angle Image
  * @brief API for Range Angle Image algorithm
  *
  * An algorithm explanation is available at the \ref ssct_radarsdk_algorithms_rai SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates a Range Angle Image handle (object), based on the input parameters
 *
 * @param [in] config        Range Angle Image configurations defined by \ref ifx_RAI_Config_t.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_RAI_Handle_t ifx_rai_create(ifx_RAI_Config_t* config);

/**
 * @brief Calculates range angle image from real input raw data.
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 * @param [in]     input     An array of real 2D raw ADC data matrices for all Rx channels i.e.
 *                           (Number of Antennas x NumChirps x Nsamples)
 *
 * @param [out]    output    An arrays of real 2D matrices for range Doppler image beams i.e.
 *                           (NumImages x Nsamples x NumberofBeams)
 */
IFX_DLL_PUBLIC
void ifx_rai_run_r(ifx_RAI_Handle_t handle,
                   const ifx_Cube_R_t* input,
                   ifx_Cube_R_t* output);


/**
 * @brief Calculates range angle image from complex (IQ) input raw data.
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 * @param [in]     input     An array of complex 2D raw ADC data matrices for all Rx channels i.e.
 *                           (Number of Antennas x NumChirps x Nsamples)
 *
 * @param [out]    output    An arrays of real 2D matrices for range Doppler image beams i.e.
 *                           (NumImages x Nsamples x NumberofBeams)
 *
 */
IFX_DLL_PUBLIC
void ifx_rai_run_cr(ifx_RAI_Handle_t handle,
                    const ifx_Cube_C_t* input,
                    ifx_Cube_R_t* output);

/**
 * @brief Performs destruction of Range Angle Image handle (object) to clear internal states and memories.
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 *
 */
IFX_DLL_PUBLIC
void ifx_rai_destroy(ifx_RAI_Handle_t handle);

/**
 * @brief Getter function to access SNR result
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 * @return Pointer to SNR result
 */
IFX_DLL_PUBLIC
ifx_Vector_R_t* ifx_rai_get_snr(ifx_RAI_Handle_t handle);

/**
 * @brief Getter function to access RX spectrum result
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 * @return Pointer to RX spectrum result
 */
IFX_DLL_PUBLIC
ifx_Cube_C_t* ifx_rai_get_rx_spectrum(ifx_RAI_Handle_t handle);

/**
 * @brief Getter function to access Range Doppler result
 *
 * @param [in]     handle    A handle to the Range Angle Image object
 * @return Pointer to Range Doppler result
 */
IFX_DLL_PUBLIC
ifx_Cube_C_t* ifx_rai_get_range_doppler(ifx_RAI_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_RANGE_ANGLE_IMAGE_H */

/**
* @}
*/