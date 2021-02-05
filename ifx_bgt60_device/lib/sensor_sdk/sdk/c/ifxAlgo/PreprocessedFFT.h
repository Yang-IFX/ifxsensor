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
 * @file PreprocessedFFT.h
 *
 * \brief \copybrief gr_ppfft
 *
 * For details refer to \ref gr_ppfft
 */

#ifndef IFX_ALGO_PREPROCESSED_FFT_H
#define IFX_ALGO_PREPROCESSED_FFT_H

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

#include "ifxAlgo/FFT.h"
#include "ifxAlgo/Window.h"

#include "ifxBase/Math.h"
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
 * @brief Defines the structure for pre-processed FFT related settings.
 */
typedef struct
{
    ifx_FFT_Type_t      fft_type;               /**< FFT type is defined by \ref ifx_FFT_Type_t. 
                                                     Type is defined based on FFT input data nature, i.e. real or complex.*/
    uint32_t            fft_size;               /**< FFT size must be power of 2 else returns error.*/

    bool                mean_removal_enabled;      /**< Zero value considered as False (skip mean removal feature), while non-zero value would
                                                     be considered as true for mean removal.*/
    ifx_Window_Config_t window_config;          /**< Window type, length and attenuation used for range FFT.*/

    bool                is_normalized_window;   /**< Setting to zero will turn off this feature. If set to non-zero value the effect of different
                                                     window type is normalized. Changing window type will result in similar magnitude range.*/
} ifx_PPFFT_Config_t;

/**
 * @brief A handle for an instance of Pre-processed FFT module, see PPFFT.h.
 */
typedef struct ifx_PPFFT_s* ifx_PPFFT_Handle_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Signal_Processing
  * @{
  */
 
/** @defgroup gr_ppfft Pre-processed FFT
  * @brief API for Pre-processed FFT
  *
  *
  * Pre-processed FFT is a 1D FFT along with pre-processing like mean removal
  * and windowing.
  * Following list of signal processing blocks are used within this module:
  *    1. mean removal (optional)
  *    2. windowing
  *    3. FFT (R2C or C2C)
  *
  * @{
  */

/**
 * @brief Creates a handle (object) for 1D FFT chain, containing all required settings and internal memories.
 *
 * @param [in]     config    Mean removal flag and FFT settings e.g. FFT type and FFT size
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_PPFFT_Handle_t ifx_ppfft_create(const ifx_PPFFT_Config_t* config);

/**
 * @brief Calculates 1D FFT for real input with some pre-processing steps like mean removal and windowing.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 * @param [in]     input     Real input vector (single chirp data with either I or Q samples)
 * @param [out]    output    FFT output is always complex. But only half of the output
 *                           is useful as next half is just conjugate symmetric part of first half
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_run_rc(ifx_PPFFT_Handle_t handle,
                      const ifx_Vector_R_t* input,
                      ifx_Vector_C_t* output);

/**
 * @brief Calculates 1D FFT for complex input with some pre-processing steps like mean removal and windowing.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 * @param [in]     input     Complex input vector (single chirp data with both I & Q samples)
 * @param [out]    output    FFT output is always complex, and full spectrum of FFT size for complex input
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_run_c(ifx_PPFFT_Handle_t handle,
                     const ifx_Vector_C_t* input,
                     ifx_Vector_C_t* output);

/**
 * @brief Destroys handle (object) for 1D FFT chain along with internal memories.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_destroy(ifx_PPFFT_Handle_t handle);

/**
 * @brief Sets the mean removal flag to true or false.
 *
 * @param [in,out] handle    A handle to the 1D pre-processed FFT object
 * @param [in]     flag      Zero means false, else its true. Mean is calculated and removed before FFT
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_set_mean_removal_flag(ifx_PPFFT_Handle_t handle,
                                     const uint8_t flag);

/**
 * @brief Returns the current mean removal flag values used within \ref ifx_PPFFT_Handle_t.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Mean removal flag value: Zero means skip mean removal, non-zero means with mean removal.
 *
 */
IFX_DLL_PUBLIC
uint8_t ifx_ppfft_get_mean_removal_flag(const ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns FFT size in power of 2 within [16, 1024].
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return FFT size in power of 2.
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_ppfft_get_fft_size(const ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns FFT type R2C or C2C.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return FFT type real or complex inputs.
 */
IFX_DLL_PUBLIC
ifx_FFT_Type_t ifx_ppfft_get_fft_type(const ifx_PPFFT_Handle_t handle);

/**
 * @brief Facilitates to update window parameters used before FFT operation.
 *
 * For example, if window type or its scale needs to be modified, one can update by passing the
 * new window type or attenuation scale in window setting structure defined by \ref ifx_Window_Config_t.
 *
 * @param [in]     config    Window configuration defined by \ref ifx_Window_Config_t
 *                           with new gain value or window type
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_set_window(ifx_PPFFT_Handle_t handle,
                          const ifx_Window_Config_t* config);

/**
 * @brief Returns pointer to the window used in preprocessed FFT.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Pointer to the real vector containing window values
 *
 */
IFX_DLL_PUBLIC
ifx_Vector_R_t* ifx_ppfft_get_window(ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns type of window used in preprocessed FFT.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Window type defined by \ref ifx_Window_Type_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Window_Type_t ifx_ppfft_get_window_type(ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns size of window used in preprocessed FFT.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Window size i.e. number of elements.
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_ppfft_get_window_size(ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns attenuation level in dB of window used in preprocessed FFT.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Window attenuation level in dB, used only for Chebyshev window.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_ppfft_get_window_attenuation(ifx_PPFFT_Handle_t handle);

/**
 * @brief Returns configuration of window defined by \ref ifx_Window_Config_t used in preprocessed FFT.
 *
 * @param [in]     handle    A handle to the 1D pre-processed FFT object
 *
 * @return Pointer to window configuration defined by \ref ifx_Window_Config_t.
 */
IFX_DLL_PUBLIC
ifx_Window_Config_t* ifx_ppfft_get_window_config(ifx_PPFFT_Handle_t handle);

/**
 * @brief Facilitates to get the frequency axis used for plot.
 *
 * @param [in]     handle                        A handle to the 1D pre-processed FFT object
 * @param [in]     sampling_freq_Hz              Sampling frequency used to capture the signal (in units of Hz)
 * @param [out]    fft_freq_axis_spec_Hz         X-axis of FFT with Min, Max values in it defined by \ref ifx_Math_Axis_Spec_t (in units of Hz)
 *
 */
IFX_DLL_PUBLIC
void ifx_ppfft_calc_freq_axis(const ifx_PPFFT_Handle_t handle,
                             const ifx_Float_t sampling_freq_Hz,
                             ifx_Math_Axis_Spec_t* fft_freq_axis_spec_Hz);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_PREPROCESSED_FFT_H */
