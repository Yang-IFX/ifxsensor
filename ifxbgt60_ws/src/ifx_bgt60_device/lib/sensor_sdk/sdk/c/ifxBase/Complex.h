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
 * @file Complex.h
 *
 * \brief \copybrief gr_complex
 *
 * For details refer to \ref gr_complex
 */

#ifndef IFX_BASE_COMPLEX_H
#define IFX_BASE_COMPLEX_H

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

#define IFX_COMPLEX_REAL(c) ((c).data[0])
#define IFX_COMPLEX_IMAG(c) ((c).data[1])
#define IFX_COMPLEX_SET_REAL(c, r) do{ (c).data[0] = r; } while(0)
#define IFX_COMPLEX_SET_IMAG(c, i) do{ (c).data[1] = i; } while(0)
#define IFX_COMPLEX_DEF(r, i) { {r, i} }
#define IFX_COMPLEX_SET(c, r, i) \
    do { IFX_COMPLEX_SET_REAL(c, r); IFX_COMPLEX_SET_IMAG(c, i); } while(0)

IFX_DLL_PUBLIC
extern const ifx_Complex_t ifx_complex_zero;

IFX_DLL_PUBLIC
extern const ifx_Complex_t ifx_complex_one;

#define IFX_COMPLEX_IS_EQUAL(a, b) \
    (IFX_COMPLEX_REAL(a) == IFX_COMPLEX_REAL(b)) && \
    (IFX_COMPLEX_IMAG(a) == IFX_COMPLEX_IMAG(b))

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
  * @{
  */

/** @defgroup gr_complex Complex
  * @brief API for operations on complex numbers
  *
  * @{
  */

/**
 * @brief Computes the absolute value of a complex number |z|.
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed absolute value: square root of sum of squares (hypotenuse) defined by \ref ifx_Float_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_complex_abs(ifx_Complex_t z);

/**
 * @brief Computes the complex conjugate of a complex number.
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed complex conjugate defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_conj(ifx_Complex_t z);

/**
 * @brief Adds two complex numbers a + b.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Addition result of the passed complex numbers defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_add(ifx_Complex_t a,
                              ifx_Complex_t b);

/**
 * @brief Subtracts two complex numbers a - b.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Subtraction result of the passed complex numbers defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_sub(ifx_Complex_t a,
                              ifx_Complex_t b);

/**
 * @brief Applies multiplication of two complex numbers a * b.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Multiplication result of the passed complex numbers defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_mul(ifx_Complex_t a,
                              ifx_Complex_t b);

/**
 * @brief Applies division of two complex numbers a / b.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Division result of the passed complex numbers defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_div(ifx_Complex_t a,
                              ifx_Complex_t b);

/**
 * @brief Adds a real value to a complex number.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Real floating value defined by \ref ifx_Float_t.
 *
 * @return Addition result of the passed complex number and real value defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_add_real(ifx_Complex_t a,
                                   ifx_Float_t b);

/**
 * @brief Subtracts a real value from a complex number.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Real floating value defined by \ref ifx_Float_t.
 *
 * @return Subtraction result of the passed complex number and real value defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_sub_real(ifx_Complex_t a,
                                   ifx_Float_t b);

/**
 * @brief Applies multiplication by real value to a complex number.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Real floating value defined by \ref ifx_Float_t.
 *
 * @return Multiplication result of the passed complex number by a real value defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_mul_real(ifx_Complex_t a,
                                   ifx_Float_t b);

/**
 * @brief Applies division by real value to a complex number.
 *
 * @param [in]     a         Complex number defined by \ref ifx_Complex_t.
 * @param [in]     b         Real floating point values defined by \ref ifx_Float_t.
 *
 * @return Division result of the passed complex number by a real value defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_div_real(ifx_Complex_t a,
                                   ifx_Float_t b);

/**
 * @brief Computes the complex logarithm of a complex number ln(z).
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed complex logarithm the passed complex number defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_ln(ifx_Complex_t z);

/**
 * @brief Computes the complex logarithm to the base 10 of a complex number log10(z).
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed complex log10 of the passed complex number defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_log10(ifx_Complex_t z);

/**
 * @brief Computes the complex argument of a complex number.
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed complex argument of the passed complex number defined by \ref ifx_Float_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_complex_arg(ifx_Complex_t z);

/**
 * @brief Returns the polar form of a complex number.
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Polar form of the of the passed complex number defined by \ref ifx_Polar_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Polar_t ifx_complex_to_polar(ifx_Complex_t z);

/**
 * @brief Returns the complex number of a polar form.
 *
 * @param [in]     zp         Polar form of a complex number defined by \ref ifx_Polar_t.
 *
 * @return Complex number of the passed polar form defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_from_polar(ifx_Polar_t zp);

/**
 * @brief Computes the square value of a complex number (a + bi)².
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed square value of the passed complex number.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_complex_square(ifx_Complex_t z);

/**
 * @brief Computes the squared norm (absolute square) of a complex number |z|².
 *
 * @param [in]     z         Complex number defined by \ref ifx_Complex_t.
 *
 * @return Computed squared norm value of the passed complex number defined by \ref ifx_Float_t.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_complex_sqnorm(ifx_Complex_t z);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_COMPLEX_H */
