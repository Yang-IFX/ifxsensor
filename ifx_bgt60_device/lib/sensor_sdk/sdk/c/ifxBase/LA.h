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
* \file LA.h
*
* \brief   \copybrief gr_la
*
* For details refer to \ref gr_la
*
*
* @{
*/


#ifndef IFX_BASE_LA_H
#define IFX_BASE_LA_H

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

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
  * @{
  */
 
/** @defgroup gr_la Linear Algebra
  * @brief API for Linear Algebra (LA)
  *
  * Supports linear algebra operations such as LU,
  * Cholesky decomposition, or inverting matrices
  *
  * @{
  */

/**
 * @brief Computes inverse of a generic real matrix
 *
 * This function computes the inverse matrix of A (\f$A^{-1}\f$) and saves it in Ainv.
 *
 * This function works for generic real square matrices. If A is (numerically)
 * singular, IFX_ERROR_MATRIX_SINGULAR is set.
 *
 * @param [in]     A         input matrix A
 * @param [out]    Ainv      inverse of matrix A
 *
 */
IFX_DLL_PUBLIC
void ifx_la_invert_r(const ifx_Matrix_R_t* A,
                     ifx_Matrix_R_t* Ainv);

/**
 * @brief Computes inverse of a generic complex matrix
 *
 * This function computes the inverse matrix of A (\f$A^{-1}\f$) and saves it in Ainv.
 *
 * This function works for generic complex square matrices. If A is
 * (numerically) singular, IFX_ERROR_MATRIX_SINGULAR is set.
 *
 * @param [in]     A         input matrix A
 * @param [out]    Ainv      inverse of matrix A
 *
 */
IFX_DLL_PUBLIC
void ifx_la_invert_c(const ifx_Matrix_C_t* A,
                     ifx_Matrix_C_t* Ainv);

/**
 * @brief Performs Cholesky decomposition of A
 *
 * Compute the Cholesky decomposition of the real symmetric and positive
 * definite matrix A
 * \f[
 *      A = L L^\mathrm{T},
 * \f]
 * where L is a real lower triangular matrix with positive diagonal elements.
 * Only the lower triangular part of A (matrix elements on the diagonal and
 * below the diagonal) are read.
 *
 * If A is not positive definite, IFX_ERROR_MATRIX_NOT_POSITIVE_DEFINITE is
 * set.
 *
 * @param [in]     A         matrix A
 * @param [out]    L         lower triangular matrix
 *
 */
IFX_DLL_PUBLIC
void ifx_la_cholesky_r(const ifx_Matrix_R_t* A,
                       ifx_Matrix_R_t* L);

/**
 * @brief Performs Cholesky decomposition of A
 *
 * Compute the Cholesky decomposition of the hermitian and positive definite
 * matrix A
 * \f[
 *      A = L L^\dagger,
 * \f]
 * where L is a complex lower triangular matrix with real and positive diagonal
 * elements. Only the lower triangular part of A (matrix elements on the
 * diagonal and below the diagonal) are read.
 *
 * If A is not positive definite, IFX_ERROR_MATRIX_NOT_POSITIVE_DEFINITE is
 * set.
 *
 * @param [in]     A         matrix A
 * @param [out]    L         lower triangular matrix
 *
 */
IFX_DLL_PUBLIC
void ifx_la_cholesky_c(const ifx_Matrix_C_t* A,
                       ifx_Matrix_C_t* L);

/**
 * @brief Computes determinant of a generic real matrix
 *
 * This function computes the determinant of A and saves it in determinant.
 *
 * @param [in]     A                   matrix
 * @param [out]    determinant         determinant of A
 *
 */
IFX_DLL_PUBLIC
void ifx_la_determinant_r(const ifx_Matrix_R_t* A,
                          ifx_Float_t* determinant);

/**
 * @brief Computes determinant of a generic complex matrix
 *
 * This function computes the determinant of A and saves it in determinant.
 *
 * @param [in]     A                   matrix
 * @param [out]    determinant         determinant of A
 *
 */
IFX_DLL_PUBLIC
void ifx_la_determinant_c(const ifx_Matrix_C_t* A,
                          ifx_Complex_t* determinant);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_LA_H */

/**
* @}
*/