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
* \file 2DMTI.h
*
* \brief   \copybrief gr_mti2d
*
* For details refer to \ref gr_mti2d
*
*
* @{
*/


#ifndef IFX_ALGO_2DMTI_H
#define IFX_ALGO_2DMTI_H

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
 * @brief Forward declaration structure for 2D MTI filter to operate on real Matrix.
 */
typedef struct ifx_2DMTI_R_s* ifx_2DMTI_Handle_R_t;

/**
 * @brief Forward declaration structure for 2D MTI filter to operate on complex Matrix.
 */
typedef struct ifx_2DMTI_C_s* ifx_2DMTI_Handle_C_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_mti2d Moving Target Indicator 2D (2DMTI)
  * @brief API for 2D Moving Target Indicator filter (2DMTI)
  *
  * The Moving Target Indicator (MTI) is a radar targeting method that helps
  * discriminating moving targets from static targets and clutter.
  *
  * The formulae are equivalent to the 1D MTI case. For more information refer to
  * the documentation of \ref gr_mti.
  *
  * An algorithm explanation is also available at the \ref ssct_radarsdk_algorithms_2dmti SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates 2D MTI filter handle to operate on real matrix.
 *
 * @param [in]     alpha_mti_filter    Scalar for 2D MTI Filter parameter. Valid range [0.0, 1.0]
 * @param [in]     rows                Number of rows of matrix of FFT spectrum used within 2D MTI filter
 * @param [in]     columns             Number of columns of matrix of FFT spectrum used within 2D MTI filter
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 */
IFX_DLL_PUBLIC
ifx_2DMTI_Handle_R_t ifx_2dmti_create_r(const ifx_Float_t alpha_mti_filter,
                                        const uint32_t rows,
                                        const uint32_t columns);

/**
 * @brief Creates 2D MTI filter handle to operate on complex matrix.
 *
 * @param [in]     alpha_mti_filter    Scalar for 2D MTI Filter parameter. Valid range [0.0, 1.0]
 * @param [in]     rows                Number of rows of matrix of FFT spectrum used within 2D MTI filter
 * @param [in]     columns             Number of columns of matrix of FFT spectrum used within 2D MTI filter
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 */
IFX_DLL_PUBLIC
ifx_2DMTI_Handle_C_t ifx_2dmti_create_c(const ifx_Float_t alpha_mti_filter,
                                        const uint32_t rows,
                                        const uint32_t columns);

/**
 * @brief Destroys the 2D MTI filter handle for real Matrix.
 *
 * @param [in,out] handle    A handle to the 2D MTI filter to operate on real matrix
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_destroy_r(ifx_2DMTI_Handle_R_t handle);

/**
 * @brief Destroys the 2D MTI filter handle for complex Matrix.
 *
 * @param [in,out] handle    A handle to the 2D MTI filter to operate on complex matrix
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_destroy_c(ifx_2DMTI_Handle_C_t handle);

/**
 * @brief Removes static parts from real output using 2D MTI filtering.
 *
 * @param [in]     handle    A handle to the 2D MTI filter to operate on real matrix
 * @param [in]     input     Real value matrix used as an input for 2D MTI filter
 * @param [out]    output    Real value matrix used as an output of 2D MTI filter
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_run_r(ifx_2DMTI_Handle_R_t handle,
                     const ifx_Matrix_R_t* input,
                     ifx_Matrix_R_t* output);

/**
 * @brief Removes static parts from complex output using 2D MTI filtering.
 *
 * @param [in]     handle    A handle to the 2D MTI filter to operate on complex matrix
 * @param [in]     input     Complex value matrix used as an input for 2D MTI filter
 * @param [out]    output    Complex value matrix used as an output of 2D MTI filter
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_run_c(ifx_2DMTI_Handle_C_t handle,
                     const ifx_Matrix_C_t* input,
                     ifx_Matrix_C_t* output);

/**
 * @brief Runtime modification of 2D MTI filter scalar coefficient on real matrix.
 *
 * This will not reset the history as after some frames history will converge to
 * new filter coefficient.
 *
 * @param [in]     handle              A handle to the 2D MTI filter for real matrix operation
 * @param [in]     alpha_mti_filter    Scalar filter coefficient for 2D MTI Filter. Valid range [0.0, 1.0]
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_set_filter_coeff_r(ifx_2DMTI_Handle_R_t handle,
                                  const ifx_Float_t alpha_mti_filter);

/**
 * @brief Returns currently used 2D MTI filter scalar coefficient on real matrix.
 *
 * @param [in]     handle              A handle to the 2D MTI filter for real matrix operation
 *
 * @return Scalar filter coefficient for 2D MTI Filter. Valid range [0.0, 1.0]
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_2dmti_get_filter_coeff_r(ifx_2DMTI_Handle_R_t handle);

/**
 * @brief Runtime modification of 2D MTI filter scalar coefficient on complex matrix
 *
 * This will not reset the history as after some frames history will converge to
 * new filter coefficient.
 *
 * @param [in]     handle              A handle to the 2D MTI filter for complex matrix operation
 * @param [in]     alpha_mti_filter    Scalar filter coefficient for 2D MTI Filter. Valid range [0.0, 1.0]
 *
 */
IFX_DLL_PUBLIC
void ifx_2dmti_set_filter_coeff_c(ifx_2DMTI_Handle_C_t handle,
                                  const ifx_Float_t alpha_mti_filter);

/**
 * @brief Returns currently used 2D MTI filter scalar coefficient on complex matrix
 *
 * @param [in]     handle              A handle to the 2D MTI filter for complex matrix operation
 *
 * @return Scalar filter coefficient for 2D MTI Filter. Valid range [0.0, 1.0]
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_2dmti_get_filter_coeff_c(ifx_2DMTI_Handle_C_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_2DMTI_H */

/**
* @}
*/