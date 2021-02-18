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
* \file Signal.h
*
* \brief   \copybrief gr_signal
*
* For details refer to \ref gr_signal
*
*
* @{
*/


#ifndef IFX_ALGO_FILTER_H
#define IFX_ALGO_FILTER_H

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
 * @brief Forward declaration structure for linear filter to operate on Real signal Vector
 */
typedef struct ifx_Filter_R_s ifx_Filter_R_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
* @{
*/

/** @defgroup gr_signal signal
* @brief API for digital filtering operations.
*
* Supports mathematical and other operations such as creation and destruction of
* filter objects, filtering operation and clearing filter state history.
*
* @{
*/

/**
* @brief IIR FIR Linear Filter
* Filters a data sequence, input, using a digital filter with M
* numerator coefficients vector b, and N denominator coefficients vector a.
* The initial states (past samples of input and output) are set to zero.
* The filter is implemented as a direct II transposed structure.
* The formula for calculating the nth sample of output
* is expressed as
* \f[
* \mathrm{output}_n = \frac{1}{a_0} \left(\sum_{j=0}^{M-1} b_j \cdot \mathrm{input}_{n-j} - 
*                     \sum_{j=1}^{N-1}a_j \cdot \mathrm{output}_{n-j}\right)
* \f]
*
*
* @param [in]     filter    Filter structure which includes the following pointers
*                              - b         vector : the numerator coefficient vector
*                              - a         vector : the denominator coefficient vector
*                                              If a[0] is not 1, then the other coefficients are normalized by it's value.
*                              - state_b   vector : FIR state from the last filtering operation
*                              - state_a   vector : IIR state from the last filtering operation
*
* @param [in]     input     input signal vector
*
* @param [out]    output    output vector(should be the same length as input)
*
*/
IFX_DLL_PUBLIC
void ifx_signal_filt_run_r(ifx_Filter_R_t* filter,
                    const ifx_Vector_R_t* input,
                    ifx_Vector_R_t* output);

/**
* @brief Allocate memory and initialize a filter object with numerator and
* denominator coefficients. Resets filter states to zero.
*
*
* @param [in]     b         ifx_vector containing
*                           numerator coefficients to be loaded to filter
*
* @param [in]     a         ifx_vector containing
*                           denominator coefficients to be loaded to filter
*
* @return Pointer to allocated and initialized real Filter structure
*
*/
IFX_DLL_PUBLIC
ifx_Filter_R_t* ifx_signal_filt_create_r(const ifx_Vector_R_t* b,
                                  const ifx_Vector_R_t* a);

/**
* @brief Resets filter states maintaining the filter coefficients
*
*
* @param [in,out] filter    filter stucture whose states are
*                           to be reset.
*
*/
IFX_DLL_PUBLIC
void ifx_signal_filt_reset_r(ifx_Filter_R_t* filter);

/**
* @brief Frees the memory allocated for a Filter object \ref ifx_Filter_R_t
*
*
* @param [in]     filter    filter stucture whose memory and members need to be
*                           deallocated.
*
*/
IFX_DLL_PUBLIC
void ifx_signal_filt_destroy_r(ifx_Filter_R_t* filter);

/**
 * @brief Creates a correlation vector using 2 input vectors.
 * Cross-correlate two 1-dimensional arrays in1 and in2. 
 * The output 'out' is the same size as 'in1'. For a Signal (in1) of length 'N' and a 
 * Vector (in2) of length 'M' the function can be expressed as three regions
 * explaining the handling of odd and even values of 'M'.
 * \f[
 * \mathrm{output}_n = \sum_{j=0}^{\left\lceil{\frac{M}{2}}\right\rceil+n-1} 
 *                     \mathrm{Vector}_{\left\lfloor{\frac{M}{2}}\right\rfloor-n+j} 
 *                     \cdot \mathrm{Signal}_j\quad,
 *                     \qquad \left\{ 0 \le n \le \left(\left\lfloor{\frac{M}{2}}\right\rfloor - 1\right)\right\}
 * \f]
 * \f[
 * \mathrm{output}_n = \sum_{j=0}^{M-1}
 *                     \mathrm{Vector}_{j}
 *                     \cdot \mathrm{Signal}_{n-\left\lfloor{\frac{M}{2}}\right\rfloor+j}\quad,
 *                     \qquad \left\{ \left(\left\lfloor{\frac{M}{2}}\right\rfloor\right) \le n \le 
 *                     \left( N - \left\lceil{\frac{M}{2}}\right\rceil \right)\right\}
 * \f]
 * \f[
 * \mathrm{output}_n = \sum_{j=0}^{\left\lfloor{\frac{M}{2}}\right\rfloor + N -(n+1)}
 *                     \mathrm{Vector}_{j}
 *                     \cdot \mathrm{Signal}_{n-\left\lfloor{\frac{M}{2}}\right\rfloor+j}\quad,
 *                     \qquad \left\{ \left( N - \left\lceil{\frac{M}{2}}\right\rceil +1\right) \le n \le
 *                     \left( N - 1\right)\right\}
 * \f]
 *
 * @param [in]     in1    Pointer to input signal vector defined by \ref ifx_Vector_R_t
 *
 * @param [in]     in2    Pointer to correlation vector defined by \ref ifx_Vector_R_t
 *
 * @param [out]    out    Pointer to output vector defined by \ref ifx_Vector_R_t
 *                        (should be equal in length to 'in1' input signal vector.)
 *
 */
IFX_DLL_PUBLIC
void ifx_signal_correlate_r(const ifx_Vector_R_t* in1,
    const ifx_Vector_R_t* in2,
    ifx_Vector_R_t* out);

/**
 * @brief Generates a gaussian pulse vector.
 * Uses pulse configuration parameters \f$b_w\f$ (pulse bandwidth) and \f$f_c\f$(center frequency)
 * The output 'output' is the same size as 't'. The output is a sinusoid shaped by a
 * gaussian function envelope. The \f$\mathrm{bwr}\f$ or reference level at which fractional bandwidth
 * is calculated is fixed at -6 (dB). The elements of the output are populated according to
 * the following equation
 * \f[
 * \mathrm{output}_n = e^{-\mathrm{a} {t_n}^{2}} \cdot cos(2 \pi f_c t_n)
 * \f]
 * where
 * \f[
 * \mathrm{a} = \frac{-(\pi f_c b_w)^2}{4 \mathrm{ln}(10^{\frac{bwr}{20}})}
 * \f]
 *
 * @param [in]     t            time vector defined by \ref ifx_Vector_R_t
 *
 * @param [in]     fc           pulse center frequecy in Hz
 *
 * @param [in]     bw           pulse bandwidth
 *
 * @param [out]    output       output vector defined by \ref ifx_Vector_R_t
 *                              contains gaussian pulse values.
 *                              (should be equal in length to 't' input time vector.)
 *
 */
IFX_DLL_PUBLIC
void ifx_signal_gaussianpulse_r(const ifx_Vector_R_t* t,
    const ifx_Float_t fc,
    const ifx_Float_t bw,
    ifx_Vector_R_t* output);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_SIGNAL_H */

/**
* @}
*/