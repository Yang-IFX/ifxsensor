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
* \file Segmentation.h
*
* \brief   \copybrief gr_segmentation
*
* For details refer to \ref gr_segmentation
*
*
* @{
*/


#ifndef IFX_RADAR_SEGMENTATION_H
#define IFX_RADAR_SEGMENTATION_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdbool.h>

#include "ifxBase/Forward.h"

#include "ifxRadar/DeviceConfig.h"

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
 * \brief This enumeration type lists the detection sensitivity levels.
 */
typedef enum
{
   IFX_DETECTION_SENSITIVITY_HIGH = 1U, /**< High detection sensitivity.*/
   IFX_DETECTION_SENSITIVITY_MED  = 2U, /**< Medium detection sensitivity.*/
   IFX_DETECTION_SENSITIVITY_LOW  = 3U  /**< Low detection sensitivity.*/
} ifx_Detection_Sensitivity_t;

/**
 * \brief This enumeration type lists the static target cancelation levels.
 */
typedef enum
{
   IFX_STATIC_TARGET_CANCELATION_HIGH = 1U, /**< High static target cancelation.*/
   IFX_STATIC_TARGET_CANCELATION_MED  = 2U, /**< Medium static target cancelation.*/
   IFX_STATIC_TARGET_CANCELATION_LOW  = 3U  /**< Low static target cancelation.*/
} ifx_Static_Target_Cancelation_t;

/**
 * @brief A handle for an instance of Segmentation module, see Segmentation.h
 */
typedef struct ifx_Segmentation_s* ifx_Segmentation_Handle_t;

/**
 * @brief Defines the structure for Segmentation module related settings.
 */
typedef struct
{
    uint32_t        num_samples_per_chirp;      /**< Number of samples per chirp.*/
    uint32_t        num_chirps_per_frame;       /**< Number of chirps per frame.*/
    ifx_Float_t     bandwidth_Hz;               /**< Frequency bandwidth in Hz.*/
    ifx_Float_t     chirp_repetition_time_s;    /**< Chirp repetition time in seconds.*/
    ifx_Float_t     center_frequency_Hz;        /**< Center frequency in Hz.*/
    ifx_Float_t     max_detection_range_m;      /**< Maximum range of radar in meters.*/
    ifx_Vector_R_t* segments_degrees;           /**< Width of each segments in degrees.*/
    ifx_Detection_Sensitivity_t     detection_sensitivity; /**< Detection sensitivity.*/
    ifx_Static_Target_Cancelation_t static_target_removal; /**< Static target cancelation.*/
    bool            enable_hysteresis;          /**< false = off, true = on. */
    uint32_t        max_num_tracks;             /**< Maximum number of tracked targets */
} ifx_Segmentation_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */
 
/** @defgroup gr_segmentation Segmentation
  * @brief API for Segmentation algorithm
  *
  * Calculates presence and absence in each segment, frame by frame.
  *
  * @{
  */

/**
 * @brief Creates a Segmentation handle (object), based on the input parameters.
 *
 * @param [in]     config    Segmentation configurations defined by \ref ifx_Segmentation_Config_t.
 * @return Handle to the newly created instance or NULL in case of failure.
 */
IFX_DLL_PUBLIC
ifx_Segmentation_Handle_t ifx_segmentation_create(ifx_Segmentation_Config_t* config);

/**
 * @brief Performs the segmentation algorithm on given raw data.
 *
 * @param [in]     handle              A handle to the Segmentation object.
 * @param [in]     matrix_data         Input raw data (NumChirpsPerFrame x NumSamplesPerChirp x RxAntennas).
 * @param [out]    segments            Result of segments occupation.
 * @param [out]    tracks              Result of tracks:
 *                                     tracks[i][0]: track_id  (Start from 1)
 *                                     tracks[i][1]: range
 *                                     tracks[i][2]: angle
 *                                     tracks[i][3]: speed
 */
IFX_DLL_PUBLIC
void ifx_segmentation_run(const ifx_Segmentation_Handle_t handle,
                          const ifx_Cube_R_t* matrix_data,
                          ifx_Vector_R_t* segments,
                          ifx_Matrix_R_t* tracks);

/**
 * @brief Get default configuration for segmentation
 *
 * The default configuration uses:
 *  - max_detection_range_m: 7 meters
 *  - detection_sensitivity: medium
 *  - remove_static_targets: medium
 *  - enable_hysteresis: true
 *  - max_num_tracks: 5
 *
 * Three segments with each 40 degrees (120 degrees field of view) are set.
 * All other options are read from device_config.
 *
 * Note: You have to free the memory for the struct member segments_degrees
 * yourself by calling \ref ifx_vec_destroy_r.
 *
 * @param [in]     device_config       Pointer to device configuration structure.
 * @param [out]    segmentation_config Pointer to segmentation configuration structure.
 */
IFX_DLL_PUBLIC
void ifx_segmentation_config_get_defaults(const ifx_Device_Config_t* device_config, ifx_Segmentation_Config_t* segmentation_config);

/**
 * @brief Destroys Segmentation handle (object) to clear internal states and memories.
 *
 * @param [in]  handle      A handle to the Segmentation object.
 */
IFX_DLL_PUBLIC
void ifx_segmentation_destroy(ifx_Segmentation_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_SEGMENTATION_H */

/**
* @}
*/