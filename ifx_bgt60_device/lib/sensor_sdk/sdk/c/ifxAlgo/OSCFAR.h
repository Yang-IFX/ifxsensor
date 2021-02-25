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
* SOFTWARE.
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file OSCFAR.h
*
* \brief   \copybrief gr_oscfar
*
* For details refer to \ref gr_oscfar
*
*
* @{
*/


#ifndef IFX_ALGO_OSCFAR_H
#define IFX_ALGO_OSCFAR_H

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
 * @brief A handle for an instance of OSCFAR module, see OSCFAR.h.
 */
typedef struct ifx_OSCFAR_s* ifx_OSCFAR_Handle_t;

/**
 * @brief Defines the structure for OSCFAR module related settings.
 */
typedef struct
{
    uint8_t     win_rank;       /**< Rank of CFAR reference window.*/
    uint8_t     guard_band;     /**< Rank of CFAR guard band.*/
    ifx_Float_t sample;         /**< Constant used for setting CFAR threshold.*/
    ifx_Float_t pfa;            /**< Probability of false alarm.*/
    ifx_Float_t coarse_scalar;  /**< Used for coarse thresholding 2D feature map.*/
} ifx_OSCFAR_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_oscfar OS-CFAR
  * @brief API for 2D ordered statistic constant false alarm rate (OS-CFAR) algorithm.
  *
  * Input of this module is a 2D matrix of real values, i.e. range angle map.
  * Output is the a 2D matrix of same size as Input with filtered contents.
  *
  * An algorithm explanation is available at the \ref ssct_radarsdk_algorithms_detect_oscfar SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates a OSCFAR handle (object), based on the input parameters.
 *
 * @param [in]     config    OSCFAR configurations defined by \ref ifx_OSCFAR_Config_t.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_OSCFAR_Handle_t ifx_oscfar_create(ifx_OSCFAR_Config_t* config);

/**
 * @brief Runs OS_CFAR algorithm, based on the input parameters.
 *
 * @param [in]     handle              A handle to the OSCFAR object
 * @param [in]     feature2D           rangeAngle/rangeDoppler 2D feature maps with dimensions i.e.
 *                                     rangeAngle: (numOfSamplesPerChirp/2,numOfBeams)
 *                                     rangeDoppler: (numOfSamplesPerChirp/2,numChirpsPerFrame)
 * @param [out] detector_output        Appropriate 2D feature map with updated target indices.
 *
 */
IFX_DLL_PUBLIC
void ifx_oscfar_run(const ifx_OSCFAR_Handle_t handle,
                    ifx_Matrix_R_t* feature2D,
                    ifx_Matrix_R_t* detector_output);

/**
 * @brief Destroys OSCFAR handle (object) to clear internal states and memories.
 *
 * @param [in]     handle    A handle to the OSCFAR object
 *
 */
IFX_DLL_PUBLIC
void ifx_oscfar_destroy(ifx_OSCFAR_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_OSCFAR_H */

/**
* @}
*/