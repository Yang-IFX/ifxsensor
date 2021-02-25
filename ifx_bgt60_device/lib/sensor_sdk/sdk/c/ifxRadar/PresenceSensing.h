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
* \file PresenceSensing.h
*
* \brief   \copybrief gr_presencesensing
*
* For details refer to \ref gr_presencesensing
*
*
* @{
*/


#ifndef IFX_RADAR_PRESENCE_SENSING_H
#define IFX_RADAR_PRESENCE_SENSING_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include "ifxAlgo/Window.h"

#include "ifxBase/Forward.h"
#include "ifxBase/Matrix.h"

#include "ifxRadar/DeviceConfig.h"
#include "ifxRadar/RangeSpectrum.h"


/*
==============================================================================
   3. TYPES
==============================================================================
*/

/** @brief Current state of the presence sensing algorithm */
typedef enum
{
   IFX_PRESENCE_SENSING_PRESENT = 0U, /**< An object is present. */
   IFX_PRESENCE_SENSING_ABSENT  = 1U  /**< No object is present */
} ifx_PresenceSensing_State_t;

/**< @brief Definition for state status callback */
typedef void (*ifx_PresenceSensing_StatusCallback_t)(ifx_PresenceSensing_State_t cur_state, void* context);

/**< @brief Definition for state changed callback */
typedef void (*ifx_PresenceSensing_ChangeCallback_t)(ifx_PresenceSensing_State_t new_state, void* context);

/**
 * @brief PresenceSensing configuration
 *
 * This structure contains all relevant parameters for creating an instance of the presence
 * sensing module \ref ifx_PresenceSensing_Handle_t, and some parameters which determine the
 * operation of the presence sensing module.
 *
 * You can use \ref ifx_presence_sensing_get_config_defaults to get a default
 * configuration that can serve as a starting point.
 */
typedef struct
{
    uint32_t range_fft_size;                        /**< The size of the range FFTs to be performed
                                                         on the input frame data for presence sensing.*/
    uint32_t doppler_fft_size;                      /**< The size of the FFTs to be performed on Doppler domain
                                                         of the range spectrums obtained during presence sensing.*/
    ifx_Window_Type_t range_fft_window_type;        /**< The type of window to be used for range FFT.*/

    ifx_Float_t range_fft_window_alpha;             /**< The scaling of the window to be used for range FFT.*/

    ifx_Window_Type_t doppler_fft_window_type;      /**< The type of window to be used for Doppler FFT.*/

    ifx_Float_t doppler_fft_window_alpha;           /**< The scaling of the window to be used for Doppler FFT.*/

    ifx_Float_t mti_weight;                         /**< The weight if the \ref ifx_MTI_Handle_t used to remove static
                                                         targets during presence sensing*/
    ifx_Float_t min_detection_range_m;              /**< Targets below this distance are ignored by the presence sensing
                                                         algorithm.*/
    ifx_Float_t max_detection_range_m;              /**< Targets above this distance are ignored by the presence sensing
                                                         algorithm.*/
    ifx_Float_t range_hysteresis_percentage;        /**< Hysteresis for \ref max_detection_range_m while changing state from
                                                         presence to absence and vice versa.*/
    uint32_t presence_confirm_count;                /**< Number of consecutive presence detections in consecutive frames
                                                         for making a definitive presence detection.*/
    uint32_t absence_confirm_count;                 /**< Number of consecutive absence detections in consecutive frames
                                                         for making a definitive presence detection.*/
    ifx_Device_Config_t device_config;              /**< Configuration of the radar device to be used for
                                                         presence sensing.*/
    ifx_PresenceSensing_StatusCallback_t state_status_cb; /**< Pointer to a callback function that would be called once
                                                         every time the presence sensing state machine runs. (Ignored
                                                         if NULL) */
    ifx_PresenceSensing_ChangeCallback_t state_change_cb; /**< Pointer to a callback function that would be called once
                                                         the state changes from present to absent or the other way
                                                         round. (Ignored if NULL.)*/
    void* context_callback;                         /**< Context pointer given to the callbacks state_status_cb and state_change_cb.*/
    ifx_RS_Mode_t range_spectrum_mode;              /**< Mode of calculation of range spectrum.
                                                         (see \ref ifx_RS_Mode_t)*/
    ifx_Float_t threshold_factor_presence_peak;     /**< Decides threshold factor parameter in \ref ifx_Peak_Search_Config_t for configuring the
                                                         "presence_peak_handle" peak search module that is active in the PRESENCE state*/
    ifx_Float_t threshold_factor_absence_peak;      /**< Decides threshold factor parameter in \ref ifx_Peak_Search_Config_t for configuring the
                                                         "absence_peak_handle" peak search module that is active in the ABSENCE state [internal param] */
    ifx_Float_t threshold_factor_absence_fine_peak; /**< Decides threshold factor parameter in \ref ifx_Peak_Search_Config_t for configuring the
                                                         "absence_fine_peak_handle" peak search module that is active in the ABSENCE state and decides the
                                                         range bins of the range spectrums on which the Doppler FFT needs to be performed.*/
} ifx_PresenceSensing_Config_t;

/** @brief Handle for Presence Sensing algorithm */
typedef struct ifx_PresenceSensing_Handle ifx_PresenceSensing_Handle_t;



/**
 * @brief Result structure for Presence Sensing algorithm
 *
 * This structure is used to return the results of the presence sensing algorithm.
 *
 * Please note that the matrices for the range-Doppler map (range_doppler_map)
 * and for the range spectogram (range_spectogram) become invalid and must not
 * be used anymore once either
 * \ref ifx_presence_sensing_run or \ref ifx_presence_sensing_destroy is called.
 * If you still need the matrices, you have to create copies of the matrices.
 */
typedef struct {
   ifx_PresenceSensing_State_t cur_presence_state; /**< Current presence state.*/
   const ifx_Matrix_R_t *range_doppler_map;        /**< Range-Doppler Map, the number of valid rows of this matrix is defined by the num_targets
                                                        element of this structure. Presence sensing module does not calculate a full Range-Doppler map,
                                                        therefore num_targets must be considered when reading this matrix.*/
   const ifx_Matrix_C_t* range_spectrogram;        /**< Range Spectrogram.*/
   uint32_t num_targets;                           /**< Number of targets found in current run.*/
   ifx_Float_t target_distance_m;                  /**< Distance of strongest target from sensor m. If no target was found the value is NAN. */
   ifx_Float_t target_speed_m_s;                   /**< Speed of strongest target from sensor in m/s. If no target was found the value is NAN. */
   ifx_Float_t target_signal_strength;             /**< Signal strength of target. If no target was found the value is NAN. */
} ifx_PresenceSensing_Result_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_presencesensing Presence Sensing
  * @brief API for Presence Sensing algorithm
  *
  * Determine if an object is currently present or not.
  *
  * @{
  */

/**
 * @brief Create default configuration for presence sensing
 *
 * This function initializes the presence sensing configuration config with
 * default values. The default configuration can serve as a starting point
 * for further adjustments.
 *
 * @param [in]      device_config   Pointer to device configuration.
 * @param [out]     config          Pointer to presence sensing configuration.
 */
IFX_DLL_PUBLIC
void ifx_presence_sensing_get_config_defaults(const ifx_Device_Config_t* device_config,
                                              ifx_PresenceSensing_Config_t* config);

/**
 * @brief Creates a Presence Sensing handle (object).
 *
 * @param [in]     config    A pointer to an initialized presence sensing config structure.
 *
 * @return A handle of a presence sensing structure
 */
IFX_DLL_PUBLIC
ifx_PresenceSensing_Handle_t* ifx_presence_sensing_create(const ifx_PresenceSensing_Config_t* config);

/**
 * @brief Destroys Presence Sensing handle (object) to clear internal states and memories.
 *
 * @param [in]     handle    Pointer to a handle of a presence sensing structure
 */
IFX_DLL_PUBLIC
void ifx_presence_sensing_destroy(ifx_PresenceSensing_Handle_t* handle);

/**
 * @brief Performs the presence sensing algorithm on given raw data.
 *
 * Perform the presence sensing algorithm on the given matrix consisting of
 * raw IF frame data coming from the configured radar transceiver.
 *
 * The matrix frame_Data consists of IF frame data from the radar transceiver.
 * The number of rows of the input matrix should match the number of
 * chirps per frame and the number of columns should match the number
 * of samples per chirp expected by the \ref ifx_PresenceSensing_Handle_t
 * (given in the \ref ifx_Device_Config_t of the \ref ifx_PresenceSensing_Config_t
 * used while creating the given presence sensing handle).
 *
 * @param [in]     handle      Handle to the presence sensing object.
 * @param [in]     frame_data  Matrix consisting of IF frame data from the radar transceiver.
 * @param [out]    result      Structure containing the result of the run.
 */
IFX_DLL_PUBLIC
void ifx_presence_sensing_run(ifx_PresenceSensing_Handle_t* handle,
                              const ifx_Matrix_R_t* frame_data,
                              ifx_PresenceSensing_Result_t* result);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_PRESENCE_SENSING_H */

/**
* @}
*/