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
* \file Math.h
*
* \brief   \copybrief gr_math
*
* For details refer to \ref gr_math
*
*
* @{
*/


#ifndef IFX_BASE_MATH_H
#define IFX_BASE_MATH_H

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
 * @brief Defines supported Scale Types.
 */
typedef enum
{
    IFX_SCALE_TYPE_LINEAR = 0U,         /**< By default scale is linear for all modules.*/
    IFX_SCALE_TYPE_DECIBEL_10LOG = 10U, /**< Scale is in dB = 10xlog10().*/
    IFX_SCALE_TYPE_DECIBEL_20LOG = 20U  /**< Scale is in dB = 20xlog10().*/
} ifx_Math_Scale_Type_t;

/**
 * @brief Defines the structure for semantics of an axis that represents a physical quantity.
 * 
 * The axis can be along a vector (e.g., Frequency axis of a vector representing FFT values)
 * or even along one of the dimensions of a matrix (e.g., Range axis or the Speed
 * axis of a matrix representing a range-doppler map).
 * Structures of this type can be used to readily describe / visualize / plot a physical
 * quantity without additional calculations in a GUI.
 */
struct ifx_Math_Axis_Spec_s
{
    ifx_Float_t min_value;          /**< Minimum value depicted in the axis (can be negative).*/
    ifx_Float_t max_value;          /**< Maximum value depicted in the axis.*/
    ifx_Float_t value_bin_per_step; /**< Value bin represented by each step (sample) in the axis.*/
};

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
  * @{
  */
 
/** @defgroup gr_math Math
  * @brief API for Mathematical operations
  *
  * Supports mathematical operations such as
  * mean calculation, scalar removal from an array, element wise
  * addition, subtraction and multiplication of arrays, scalar
  * multiplication of an array, absolute value calculation, maximum
  * value in an array.
  *
  * @{
  */

/**
 * @brief Computes the maximum value in a real array along with the corresponding index.
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_R_t,
 *                           whose maximum value is to be computed
 * @param [out]    max_idx   Index corresponding to the maximum value (only set if not NULL)
 *
 * @return Maximum value in the given array
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_find_max(const ifx_Vector_R_t* input,
                              uint32_t* max_idx);

/**
 * @brief Computes the L1 Norm value (Manhattan distance) of the given vector, which is the sum
 *        of all its members.
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_R_t,
 *                           whose L1 Norm is to be computed
 *
 * @return L1 Norm value (Manhattan Distance) of the given Vector
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_calc_l1norm(const ifx_Vector_R_t* input);

/**
 * @brief Operates on \ref ifx_Vector_R_t arrays, to clip values less than given threshold.
 *
 * if input(n) < threshold
 *    output(n) = clip_value
 * else
 *    output(n) = input(n)
 *
 * @param [in]     input               Pointer to data memory defined by \ref ifx_Vector_R_t
 * @param [in]     threshold           Real value, must be greater than or equal to zero
 * @param [in]     clip_value          Real value, to be set if input values are greater than threshold
 * @param [out]    output              Pointer to output memory, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_clip_lt_threshold_r(const ifx_Vector_R_t* input,
                                      const ifx_Float_t threshold,
                                      const ifx_Float_t clip_value,
                                      ifx_Vector_R_t* output);

/**
 * @brief Operates on \ref ifx_Vector_R_t arrays, to clip values greater than given threshold.
 *
 * if input(n) > threshold
 *    output(n) = clip_value
 * else
 *    output(n) = input(n)
 *
 * @param [in]     input               Pointer to data memory defined by \ref ifx_Vector_R_t
 * @param [in]     threshold           Real value, must be greater than or equal to zero
 * @param [in]     clip_value          Real value, to be set if input values are greater than threshold
 * @param [out]    output              Pointer to output memory, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_clip_gt_threshold_r(const ifx_Vector_R_t* input,
                                      const ifx_Float_t threshold,
                                      const ifx_Float_t clip_value,
                                      ifx_Vector_R_t* output);

/**
 * @brief Operates on real value vectors, to perform log base 10 from standard math.
 * If any of the input real values are negative, the output would be a NaN as per the C99 standard.
 *
 * @param [in]     input     Vector of real values as an input
 * @param [out]    output    Vector of real values as output, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_log10_r(const ifx_Vector_R_t* input,
                          ifx_Vector_R_t* output);

/**
 * @brief Operates on complex value vectors, to perform log base 10 of complex numbers from standard math.
 *
 * @param [in]     input     Vector of complex values as an input
 * @param [out]    output    Vector of complex values as output, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_log10_c(const ifx_Vector_C_t* input,
                          ifx_Vector_C_t* output);

/**
 * @brief Operates on real scalar, to convert from linear to dB scale.
 * If any of the input real values are negative, the output would be a NaN as per the C99 standard.
 *
 * @param [in]     input     Real value as input
 * @param [in]     scale     For voltage this should be 20 i.e. 20xlog10() and for power 10 i.e. 10xlog10()
 *                           However, this is a generic math function, so scale can be any desired non zero float value
 *
 * @return dB value if input as per scale specified
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_linear_to_db(const ifx_Float_t input,
                                  const ifx_Float_t scale);

/**
 * @brief Operates on real vector arrays, to convert from linear to dB scale.
 *
 * @param [in]     input     Vector of real values as an input
 * @param [in]     scale     For voltage this should be 20 i.e. 20xlog10() and for power 10 i.e. 10xlog10()
 *                           However, this is a generic math function, so scale can be any desired non zero float value
 * @param [out]    output    Vector of real values as output, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_linear_to_db(const ifx_Vector_R_t* input,
                               const ifx_Float_t scale,
                               ifx_Vector_R_t* output);

/**
 * @brief Operates on real scalar, to convert from dB to Linear scale.
 *
 * @param [in]     input     Real value as input
 * @param [in]     scale     For voltage this should be 20 i.e. 10^(A/20) and for power 10 i.e. 10^(A/10)
 *
 * @return Real value as output
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_db_to_linear(const ifx_Float_t input,
                                  const ifx_Float_t scale);

/**
 * @brief Operates on real vector arrays, to convert from dB to linear scale.
 *
 * @param [in]     input     Vector of real values as an input
 * @param [in]     scale     For voltage this should be 20 i.e. 10^(A/20) and for power 10 i.e. 10^(A/10)
 * @param [out]    output    Vector of real values as output, could be same as input for in-place operation
 *
 */
IFX_DLL_PUBLIC
void ifx_math_vec_db_to_linear(const ifx_Vector_R_t* input,
                               const ifx_Float_t scale,
                               ifx_Vector_R_t* output);

/**
 * @brief Operates on real scalar, to compute absolute value.
 *
 * @param [in]     input     Real value as an input
 *
 * @return Computed absolute value
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_abs_r(ifx_Float_t input);

/**
 * @brief Operates on complex scalar, to compute absolute value.
 *
 * @param [in]     input     Complex value as an input
 *
 * @return Computed absolute value
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_math_abs_c(ifx_Complex_t input);

/**
 * @brief Check if two floats are almost equal
 *
 * Check if the floats a and b are either within the relative error reltol or
 * the absolute error is smaller than abstol.
 *
 * @param [in]     a         float a
 * @param [in]     b         float b
 * @param [in]     reltol    relative tolerance
 * @param [in]     abstol    absolute tolerance
 * @retval         1         if floats are within tolerances
 * @retval         0         otherwise
 */
IFX_DLL_PUBLIC
int ifx_math_isclose_r(ifx_Float_t a,
                       ifx_Float_t b,
                       ifx_Float_t reltol,
                       ifx_Float_t abstol);

/**
 * @brief Check if two complex floats are almost equal
 *
 * Check if the complex floats a and b are either within the relative error
 * reltol or the absolute error is smaller than abstol.
 *
 * @param [in]     a         float a
 * @param [in]     b         float b
 * @param [in]     reltol    relative tolerance
 * @param [in]     abstol    absolute tolerance
 * @retval         1         if floats are within tolerances
 * @retval         0         otherwise
 */
IFX_DLL_PUBLIC
int ifx_math_isclose_c(ifx_Complex_t a,
                       ifx_Complex_t b,
                       ifx_Float_t reltol,
                       ifx_Float_t abstol);

/**
 * @brief Round uint32 up to the next power of 2
 *
 * The function returns x rounded up to the next power of 2.
 *
 * The returned value y is a power of 2, is y>=x, and y<2*x.
 *
 * @param [in] x    input value
 * @retval y        x rounded up to the next power of 2
 */
IFX_DLL_PUBLIC
uint32_t ifx_math_round_up_power_of_2_uint32(uint32_t x);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_MATH_H */

/**
* @}
*/