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
 * @file OSCFAR.h
 *
 * \brief \copybrief gr_oscfar
 *
 * For details refer to \ref gr_oscfar
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
