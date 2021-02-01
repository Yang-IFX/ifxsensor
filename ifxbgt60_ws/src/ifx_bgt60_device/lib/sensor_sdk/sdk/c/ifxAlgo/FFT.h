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
 * @file FFT.h
 *
 * \brief \copybrief gr_fft
 *
 * For details refer to \ref gr_fft
 */

#ifndef IFX_ALGO_FFT_H
#define IFX_ALGO_FFT_H

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
 * @brief A handle for an instance of FFT processing module, see FFT.h.
 */
typedef struct ifx_FFT_s* ifx_FFT_Handle_t;

/**
 * @brief Defines supported FFT Types.
 */
typedef enum
{
    IFX_FFT_TYPE_R2C = 1U,  /**< Input is real and FFT output is complex.*/
    IFX_FFT_TYPE_C2C = 2U   /**< Input is complex and FFT output is complex.*/
} ifx_FFT_Type_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Signal_Processing
  * @{
  */
 
/** @defgroup gr_fft FFT
  * @brief API for Fast Fourier Transform (FFT)
  *
  * Supports one-dimensional FFT for real and complex input signals.
  *
  * @{
  */

/**
 * @brief Creates a FFT handle(object), based on the input parameters
 *        e.g. FFT type (\ref ifx_FFT_Type_t) and FFT size. This handle contains
 *        the complete twiddle table required for computing the FFT. During this
 *        handle construction, two cases have been taken into the consideration,
 *        regarding the input of the FFT as output is always complex.
 *        1. Real input FFT
 *        2. Complex input FFT
 *        In case 1, a special handle has been used to make FFT operation twice
 *        faster and memory efficient.
 *        For input data length less than the FFT size, zero padding is required. Thus, if user wants
 *        feed the input of different length than this parameter, the FFT handle should be destroyed and
 *        a new handle should be created.
 *        Error is returned if any of the parameter is not in the defined range of this enum
 *        (\ref ifx_FFT_Type_t)
 *
 * @param [in]     fft_type  FFT type is defined by \ref ifx_FFT_Type_t.
 *                           Type is defined based on FFT input nature, i.e. real or complex.
 * @param [in]     fft_size  Must be power of 2 else returns error.
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_FFT_Handle_t ifx_fft_create(const ifx_FFT_Type_t fft_type,
                                const uint32_t fft_size);

/**
 * @brief Performs destruction of FFT handle (object) to clear internal states
 *        used for a particular FFT size.
 *
 * @param [in]     handle    A handle to the FFT object
 *
 */
IFX_DLL_PUBLIC
void ifx_fft_destroy(ifx_FFT_Handle_t handle);

/**
 * @brief Performs FFT transform on a real input (I or Q) and produces a complex
 *        output IQ FFT signal.
 *
 * @param [in]     handle    A handle to the FFT object
 * @param [in]     input     Pointer to a floating point data memory containing real values
 * @param [out]    output    FFT output in complex IQ format, will contain half spectrum
 *                           if the output vector is only half the expected length.
 */
IFX_DLL_PUBLIC
void ifx_fft_run_rc(ifx_FFT_Handle_t handle,
                    const ifx_Vector_R_t* input,
                    ifx_Vector_C_t* output);

/**
 * @brief Performs FFT transform on a complex input IQ and produces a complex
 *        output IQ FFT signal.
 *
 * @param [in]     handle    A handle to the FFT object
 * @param [in]     input     Real valued data array on which FFT needs to be performed
 * @param [out]    output    FFT output in complex IQ format, will contain half spectrum
 *                           if the output vector is only half the expected length.
 */
IFX_DLL_PUBLIC
void ifx_fft_run_c(ifx_FFT_Handle_t handle,
                   const ifx_Vector_C_t* input,
                   ifx_Vector_C_t* output);

/**
 * @brief Performs shift on a FFT amplitude spectrum (real values) to bring DC bin in
 *        the center of spectrum, positive bins on right side and negative bins on left side.
 *        Input and output memories should be of same type and size.
 *
 * @param [in]     input     Real array on which shift operation needs to be performed
 * @param [out]    output    DC bin shifted to the center of the output array
 *
 */
IFX_DLL_PUBLIC
void ifx_fft_shift_r(const ifx_Vector_R_t* input,
                     ifx_Vector_R_t* output);

/**
 * @brief Performs shift on a FFT spectrum complex to bring DC bin in the center of spectrum,
 *        positive bins on right side and negative bins on left side.
 *        Input and output memories should be of same type and size.
 *
 * @param [in]     input     Complex array on which shift operation needs to be performed
 * @param [out]    output    DC bin shifted to the center of the output array
 *
 */
IFX_DLL_PUBLIC
void ifx_fft_shift_c(const ifx_Vector_C_t* input,
                     ifx_Vector_C_t* output);

/**
 * @brief Returns the FFT size within current FFT handle.
 *
 * @param [in]     handle    A handle to the FFT object
 *
 * @return FFT size configured in the FFT handle.
 */
IFX_DLL_PUBLIC
uint32_t ifx_fft_get_fft_size(const ifx_FFT_Handle_t handle);

/**
 * @brief Returns the FFT type within current FFT handle.
 *
 * @param [in]     handle    A handle to the FFT object
 *
 * @return FFT type configured in the FFT handle defined by \ref ifx_FFT_Type_t.
 *
 */
IFX_DLL_PUBLIC
ifx_FFT_Type_t ifx_fft_get_fft_type(const ifx_FFT_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_FFT_H */
