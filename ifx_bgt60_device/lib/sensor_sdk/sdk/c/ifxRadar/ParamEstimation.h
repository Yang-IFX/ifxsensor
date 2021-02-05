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
 * @file ParamEstimation.h
 *
 * \brief \copybrief gr_paramestimation
 *
 * For details refer to \ref gr_paramestimation
 */

#ifndef IFX_RADAR_PARAM_ESTIMATION_H
#define IFX_RADAR_PARAM_ESTIMATION_H

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
 * @brief A handle for an instance of Parameter Estimation module, see ParameEstimation.h.
 */
typedef struct ifx_ParamEstimation_s* ifx_ParamEstimation_Handle_t;

/**
 * @brief Defines the structure for ParamEstimation module related settings.
 */
typedef struct
{
    ifx_Float_t distance_per_bin;       /**< RangePerFFT bin based on modulation parameter.*/
    ifx_Float_t dop_freq_per_bin;       /**< Doppler frequency per Doppler FFT bin.*/
    ifx_Float_t hz_meter_per_sec;       /**< Hz to meter per second for Doppler calculation.*/
    /* The following parameters are used for AngleCapon module */
    uint16_t    chirps_per_frame;       /**< Number of chirps per frame.*/
    ifx_Float_t phase_offset_degrees;    /**< Phase offset compensation between used Rx antennas.*/
    uint8_t     range_win_size;         /**< Range window size.*/
    uint8_t     selected_rx;            /**< Select the best Rx channel for choosing proper Doppler index.*/
    uint8_t     max_num_target;         /**< maximum number of targets.*/
    uint8_t     num_virtual_antennas;   /**< Virtual number of antennas.*/
                                        /**< @internal Need to take care and also understand the relationship
                                             of physical number and virtual number @endinternal */
    uint8_t     num_beams;              /**< Number of beams.*/
    ifx_Float_t min_angle_degrees;       /**< Minimum angle on left side of FoV.*/
    ifx_Float_t max_angle_degrees;       /**< Maximum angle on right side of FoV.*/
    ifx_Float_t d_by_lambda;            /**< Ratio between antenna spacing 'd' and wavelength.*/
} ifx_ParamEstimation_Config_t;

/**
 * @brief Defines the structure for ParamEstimation module return results.
 */
typedef struct
{
    uint8_t         num_targets;
    ifx_Vector_R_t* range;
    ifx_Vector_R_t* angle;
    ifx_Vector_R_t* doppler;
    ifx_Vector_R_t* target_magnitude;
} ifx_ParamEstimation_Result_t;


/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_paramestimation Parameter Estimation
  * @brief API for Parameter estimation algorithm
  *
  * Supports the estimation of target parameters
  * regarding range, Doppler, angle and associated magnitude.
  *
  * @{
  */

/**
 * @brief Creates a ParamEstimation handle (object), based on the input parameters.
 *
 * @param [in]     config    ParamEstimation configurations defined by \ref ifx_ParamEstimation_Config_t.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_ParamEstimation_Handle_t ifx_paramestimation_create(ifx_ParamEstimation_Config_t* config);

/**
 * @brief Runs ParamEstimation algorithm, based on the input parameters.
 *
 * @param [in]     handle                        A handle to the ParamEstimation object
 * @param [in]     result_beam                   result beam
 * @param [in]     detected_target_indices       Detected target indices
 * @param [in]     cluster_indices               Cluster indices
 * @param [in]     rx_spectrum                   Rx spectrum
 * @param [out]    ret                           ParamEstimation result
 */
IFX_DLL_PUBLIC
void ifx_paramestimation_run(const ifx_ParamEstimation_Handle_t handle,
                             const ifx_Matrix_R_t* result_beam,
                             const ifx_Matrix_R_t* detected_target_indices,
                             const ifx_Vector_R_t* cluster_indices,
                             ifx_Cube_C_t* rx_spectrum,
                             ifx_ParamEstimation_Result_t* ret);

/**
 * @brief Destroys ParamEstimation handle (object) to clear internal states and memories.
 *
 * @param [in]     handle    A handle to the ParamEstimation object
 *
 */
IFX_DLL_PUBLIC
void ifx_paramestimation_destroy(ifx_ParamEstimation_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_PARAM_ESTIMATION_H */
