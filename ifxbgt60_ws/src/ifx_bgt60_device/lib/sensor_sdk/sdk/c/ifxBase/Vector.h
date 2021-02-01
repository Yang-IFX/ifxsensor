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
 * @file Vector.h
 *
 * \brief \copybrief gr_vector
 *
 * For details refer to \ref gr_vector
 */

#ifndef IFX_BASE_VECTOR_H
#define IFX_BASE_VECTOR_H

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

// Access macros -------------------------------------------------------------
#define IFX_VEC_STRIDE(v)       ((v)->stride)
#define IFX_VEC_OFFSET(v, idx)  ((idx)*(size_t)IFX_VEC_STRIDE(v))
#define IFX_VEC_LEN(v)          ((v)->len)
#define IFX_VEC_DAT(v)          ((v)->d)
#define IFX_VEC_AT(v, idx)      (IFX_VEC_DAT(v)[IFX_VEC_OFFSET(v, idx)])

// Condition check macro adaptations for Vector module -----------------------
#define IFX_VEC_BRK_DIM(v1, v2)             IFX_ERR_BRK_COND(IFX_VEC_LEN(v1) != IFX_VEC_LEN(v2), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_VEC_BRV_DIM(v1, v2, a)          IFX_ERR_BRV_COND(IFX_VEC_LEN(v1) != IFX_VEC_LEN(v2), IFX_ERROR_DIMENSION_MISMATCH, a)

#define IFX_VEC_BRK_MINSIZE(v, minsize)     IFX_ERR_BRK_COND(IFX_VEC_LEN(v) < (minsize), IFX_ERROR_DIMENSION_MISMATCH)

#define IFX_VEC_BRK_DIM_GT(vsmall, v)       IFX_ERR_BRK_COND(vLen(vsmall) > vLen(v), IFX_ERROR_DIMENSION_MISMATCH)

#define IFX_VEC_BRK_VEC_BOUNDS(v, idx)      IFX_ERR_BRK_COND(idx >= IFX_VEC_LEN(v), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)
#define IFX_VEC_BRF_VEC_BOUNDS(v, idx)      IFX_ERR_BRF_COND(idx >= IFX_VEC_LEN(v), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_VEC_BRK_VALID(m)  do {               \
        IFX_ERR_BRK_NULL(m);                     \
        IFX_ERR_BRK_ARGUMENT(vDat(m) == NULL)    \
    } while(0)
#define IFX_VEC_BRV_VALID(m, r)  do {            \
        IFX_ERR_BRV_NULL(m, r);                  \
        IFX_ERR_BRV_ARGUMENT(vDat(m) == NULL, r) \
    } while(0)

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @brief Defines the structure for a one-dimensional real data array.
 *        Use type ifx_Vector_R_t for this struct.
 *        Data length is fixed in this vector i.e. vector neither grows nor shrinks.
 */
struct ifx_Vector_R_s
{
    ifx_Float_t*    d;           /**< Pointer to floating point memory containing data values */
    uint32_t        len;         /**< Number of floating point elements in the array */
    uint32_t        stride : 31; /**< Stride of vector (address difference for consecutive elements) */
    uint8_t         owns_d : 1;  /**< Set to 1 if the vector owns its data and has to free it */
};

/**
 * @brief Defines the structure for one-dimensional complex data array.
 *        Use type ifx_Vector_C_t for this struct.
 *        Data length is fixed in this vector i.e. vector neither grows nor shrinks.
 */
struct ifx_Vector_C_s
{
    ifx_Complex_t*  d;           /**< Pointer to floating point memory containing data values */
    uint32_t        len;         /**< Number of floating point elements in the array */
    uint32_t        stride : 31; /**< Stride of vector (address difference for consecutive elements) */
    uint8_t         owns_d : 1;  /**< Set to 1 if the vector owns its data and has to free it */
};

/**
 * @brief Defines supported Vector sorting order options.
 */
typedef enum
{
    IFX_SORT_ASCENDING = 0,  /**< Sorting in Ascending order */
    IFX_SORT_DESCENDING      /**< Sorting in Descending order */
} ifx_Vector_Sort_Order_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
  * @{
  */

/** @defgroup gr_vector Vector
  * @brief API for operations on Vector data structures
  *
  * Supports mathematical and other operations such as creation and destruction of
  * vectors, or printing vector samples onto a file.
  *
  * @{
  */

/**
 * @brief Initializes a real vector \ref ifx_Vector_R_t with a data element of
 *        specified length, to prevent memory allocation on the heap.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     d         Data pointer to assign the vector
 * @param [in]     length    Number of elements
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_init_r(ifx_Vector_R_t* vector,
                    ifx_Float_t* d,
                    uint32_t length);

/**
 * @brief Initializes a complex vector \ref ifx_Vector_C_t with a data element of
 *        specified length, to prevent memory allocation on the heap.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     d         Data pointer to assign the vector
 * @param [in]     length    Number of elements
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_init_c(ifx_Vector_C_t* vector,
                    ifx_Complex_t* d,
                    uint32_t length);

/**
 * @brief ...
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     d         Data pointer to assign the vector
 * @param [in]     length    Number of elements
 * @param [in]     stride    Address difference for consecutive elements
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_rawview_r(ifx_Vector_R_t* vector,
                       ifx_Float_t* d,
                       uint32_t length,
                       uint32_t stride);

/**
 * @brief ...
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     d         Data pointer to assign the vector
 * @param [in]     length    Number of elements
 * @param [in]     stride    Address difference for consecutive elements
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_rawview_c(ifx_Vector_C_t* vector,
                       ifx_Complex_t* d,
                       uint32_t length,
                       uint32_t stride);

/**
 * @brief ...
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     source    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     offset    ...
 * @param [in]     length    Number of elements
 * @param [in]     spacing   ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_view_r(ifx_Vector_R_t* vector,
                    ifx_Vector_R_t* source,
                    uint32_t offset,
                    uint32_t length,
                    uint32_t spacing);

/**
 * @brief ...
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     source    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     offset    ...
 * @param [in]     length    Number of elements
 * @param [in]     spacing   ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_view_c(ifx_Vector_C_t* vector,
                    ifx_Vector_C_t* source,
                    uint32_t offset,
                    uint32_t length,
                    uint32_t spacing);

/**
 * @brief Allocates memory for a real vector for a specified number of
 *        elements and initializes it to zero.
 *
 * @param [in]     length    Number of elements in the array
 *
 * @return Pointer to allocated and initialized real vector structure or NULL if allocation failed.
 *
 */
IFX_DLL_PUBLIC
ifx_Vector_R_t* ifx_vec_create_r(uint32_t length);

/**
 * @brief Allocates memory for a real vector for a specified number of
 *        elements and initializes it to zero.
 *
 * @param [in]     length    Number of elements in the array
 *
 * @return Pointer to allocated and initialized complex vector structure or NULL if allocation failed.
 *
 */
IFX_DLL_PUBLIC
ifx_Vector_C_t* ifx_vec_create_c(uint32_t length);

/**
 * @brief Clones a real vector \ref ifx_Vector_R_t
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 *
 * @return Cloned real data array \ref ifx_Vector_R_t
 *
 */
IFX_DLL_PUBLIC
ifx_Vector_R_t* ifx_vec_clone_r(const ifx_Vector_R_t* vector);

/**
 * @brief Clones a complex vector array \ref ifx_Vector_C_t
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 *
 * @return Cloned complex data array \ref ifx_Vector_C_t
 *
 */
IFX_DLL_PUBLIC
ifx_Vector_C_t* ifx_vec_clone_c(const ifx_Vector_C_t* vector);

/**
 * @brief De-initializes a real vector \ref ifx_Vector_R_t
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_deinit_r(ifx_Vector_R_t* vector);

/**
 * @brief De-initializes a complex vector array \ref ifx_Vector_R_t
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_deinit_c(ifx_Vector_C_t* vector);

/**
 * @brief Frees the memory allocated for a real vector for a specified
 *        number of elements.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_destroy_r(ifx_Vector_R_t* vector);

/**
 * @brief Frees the memory allocated for a real vector for a specified number of
 *        elements.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_destroy_c(ifx_Vector_C_t* vector);

/**
 * @brief Blits elements of a given real data array, to a new created real data array
 *        with user defined length.
 *
 * @param [in]     vector              Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     offset              ...
 * @param [in]     length              ...
 * @param [in]     target_offset       ...
 * @param [out]    target              ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_blit_r(const ifx_Vector_R_t* vector,
                    uint32_t offset,
                    uint32_t length,
                    uint32_t target_offset,
                    ifx_Vector_R_t* target);

/**
 * @brief Blits elements of a given complex data array, to a new created complex data array
 *        with user defined length.
 *
 * @param [in]     vector              Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     offset              ...
 * @param [in]     length              ...
 * @param [in]     target_offset       ...
 * @param [out]    target              ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_blit_c(const ifx_Vector_C_t* vector,
                    uint32_t offset,
                    uint32_t length,
                    uint32_t target_offset,
                    ifx_Vector_C_t* target);
/**
 * @brief Copies the elements from a real data source array to a real data destination array.
 *
 * @param [in]     vector    Pointer to the memory containing source array defined by \ref ifx_Vector_R_t
 * @param [out]    target    Pointer to the memory containing destination array defined by \ref ifx_Vector_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_copy_r(const ifx_Vector_R_t* vector,
                    ifx_Vector_R_t* target);

/**
 * @brief Copies the elements from a complex vector source array to a complex vector destination array.
 *
 * @param [in]     vector    Pointer to the memory containing source array defined by \ref ifx_Vector_C_t
 * @param [out]    target    Pointer to the memory containing destination array defined by \ref ifx_Vector_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_copy_c(const ifx_Vector_C_t* vector,
                    ifx_Vector_C_t* target);

/**
 * @brief Performs rotation on a real vector elements by factor and elements shifted out are fed back to the
 *        vector like a circular ring.
 *        Input and output memories should be of same type and size, else error code is returned.
 *
 * @param [in]     input     Real array on which shift operation needs to be performed
 * @param [in]     shift     Rotation factor (+ive for rotate clockwise, -ive not supported yet)
 *                           elements shifted by this factor in a circular ring manner
 * @param [out]    output    Rotated vector by a given rotation factor
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_copyshift_r(const ifx_Vector_R_t* input,
                         uint32_t shift,
                         ifx_Vector_R_t* output);
/**
 * @brief Performs rotation on a complex vector elements by factor and elements shifted out are fed back to the
 *        vector like a circular ring.
 *        Input and output memories should be of same type and size, else error code is returned.
 *
 * @param [in]     input     Real array on which shift operation needs to be performed
 * @param [in]     shift     Rotation factor (+ive for rotate clockwise, -ive not supported yet)
 *                           elements shifted by this factor in a circular ring manner
 * @param [out]    output    Rotated vector by a given rotation factor
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_copyshift_c(const ifx_Vector_C_t* input,
                         uint32_t shift,
                         ifx_Vector_C_t* output);

/**
 * @brief ...
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     shift     Number of vector elements to be shifted
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_shift_r(ifx_Vector_R_t* vector,
                     uint32_t shift);

/**
 * @brief ...
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     shift     Number of vector elements to be shifted
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_shift_c(ifx_Vector_C_t* vector,
                     uint32_t shift);

/**
 * @brief Sets a real user's defined value for all real vector elements.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     value     User real defined value defined by \ref ifx_Float_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setall_r(ifx_Vector_R_t* vector,
                      ifx_Float_t value);

/**
 * @brief Sets a complex user's defined value for all complex vector elements.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     value     User complex defined value defined by \ref ifx_Complex_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setall_c(ifx_Vector_C_t* vector,
                      ifx_Complex_t value);

/**
 * @brief ...
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 * @param [in]     offset    ...
 * @param [in]     length    ...
 * @param [in]     value     ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setrng_r(ifx_Vector_R_t* vector,
                      uint32_t offset,
                      uint32_t length,
                      ifx_Float_t value);

/**
 * @brief ...
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 * @param [in]     offset    ...
 * @param [in]     length    ...
 * @param [in]     value     ...
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setrng_c(ifx_Vector_C_t* vector,
                      uint32_t offset,
                      uint32_t length,
                      ifx_Complex_t value);

/**
 * @brief Sets a real user defined value at a given index in a real vector.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
 * @param [in]     idx       Location where the value is to be set.
 * @param [in]     value     User real defined value defined by \ref ifx_Float_t.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setat_r(ifx_Vector_R_t* vector,
                     uint32_t idx,
                     ifx_Float_t value);

/**
 * @brief Sets a real user defined value at a given index in a complex vector.
 *
 * @param [in,out] vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t.
 * @param [in]     idx       Location where the value is to be set.
 * @param [in]     value     User complex defined value defined by \ref ifx_Complex_t.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_setat_c(ifx_Vector_C_t* vector,
                     uint32_t idx,
                     ifx_Complex_t value);

/**
 * @brief Computes the sum of values of a real vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
 *
 * @return Sum of all elements of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_sum_r(const ifx_Vector_R_t* vector);

/**
 * @brief Computes the sum of values of a complex vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t.
 *
 * @return Sum of all elements of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_vec_sum_c(const ifx_Vector_C_t* vector);

/**
 * @brief Computes the sum of squared values of a given real vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
 *
 * @return Sum of squared values of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_sqsum_r(const ifx_Vector_R_t* vector);

/**
 * @brief Computes the sum of squared values of a given complex vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t.
 *
 * @return Sum of squared values of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_sqsum_c(const ifx_Vector_C_t* vector);

/**
 * @brief Returns the biggest absolute value of a given real vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
 *
 * @return Maximum absolute value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_maxabs_r(const ifx_Vector_R_t* vector);

/**
 * @brief Returns the index of maximum value of a given vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
 *
 * @return Index of maximum value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_vec_max_idx_r(const ifx_Vector_R_t* vector);

/**
* @brief Returns the index of minimum value of a given vector.
*
* @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t.
*
* @return Index of minimum value of the passed input vector.
*
*/
IFX_DLL_PUBLIC
uint32_t ifx_vec_min_idx_r(const ifx_Vector_R_t* vector);

/**
 * @brief Returns the index of maximum absolute value of a given complex vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t.
 *
 * @return Index of Maximum absolute value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_maxabs_c(const ifx_Vector_C_t* vector);

/**
 * @brief Returns the maximum absolute value of a given complex vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t.
 *
 * @return Maximum absolute value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_vec_max_idx_c(const ifx_Vector_C_t* vector);

/**
 * @brief Computes element wise addition of two real vectors and stores it
 *        in result vector. This function allows in-place calculation, that
 *        means a input vector can also be the output vector at the same time
 *        if the input vector can be overwritten.
 *
 * @param [in]     v1        Pointer to operand 1, a real vector of type
 *                           \ref ifx_Vector_R_t.
 * @param [in]     v2        Pointer to operand 2, a real vector of type
 *                           \ref ifx_Vector_R_t.
 * @param [out]    result    Pointer to output vector the sum is stored to.
 *                           Can be one of the input vectors v1 or v2.
 */
IFX_DLL_PUBLIC
void ifx_vec_add_r(const ifx_Vector_R_t* v1,
                   const ifx_Vector_R_t* v2,
                   ifx_Vector_R_t* result);
/**
 * @brief Computes element wise addition of two complex vectors and stores it
 *        in result vector. This function allows in-place calculation, that
 *        means a input vector can also be the output vector at the same time
 *        if the input vector can be overwritten.
 *
 * @param [in]     v1        Pointer to operand 1, a real vector of type
 *                           \ref ifx_Vector_C_t.
 * @param [in]     v2        Pointer to operand 2, a real vector of type
 *                           \ref ifx_Vector_C_t.
 * @param [out]    result    Pointer to output vector the sum is stored to.
 *                           Can be one of the input vectors v1 or v2.
 */
IFX_DLL_PUBLIC
void ifx_vec_add_c(const ifx_Vector_C_t* v1,
                   const ifx_Vector_C_t* v2,
                   ifx_Vector_C_t* result);
/**
 * @brief Computes element wise real vector subtraction result = v1 - v2.
 *
 * @param [in]     v1        Pointer to left hand operand \ref ifx_Vector_R_t
 *                           on which element wise subtraction is applied to.
 * @param [in]     v2        Pointer to right hand operand \ref ifx_Vector_R_t
 *                           which elements are subtracted from v1.
 * @param [out]    result    Pointer to result vector \ref ifx_Vector_R_t
 *                           containing the results of element wise subtraction.
 */
IFX_DLL_PUBLIC
void ifx_vec_sub_r(const ifx_Vector_R_t* v1,
                   const ifx_Vector_R_t* v2,
                   ifx_Vector_R_t* result);
/**
 * @brief Computes element wise real vector subtraction result = v1 - v2.
 *
 * @param [in]     v1        Pointer to left hand operand \ref ifx_Vector_C_t
 *                           on which element wise subtraction is applied to.
 * @param [in]     v2        Pointer to right hand operand \ref ifx_Vector_C_t
 *                           which elements are subtracted from v1.
 * @param [out]    result    Pointer to result vector \ref ifx_Vector_C_t
 *                           containing the results of element wise subtraction.
 */
IFX_DLL_PUBLIC
void ifx_vec_sub_c(const ifx_Vector_C_t* v1,
                   const ifx_Vector_C_t* v2,
                   ifx_Vector_C_t* result);
/**
 * @brief Computes element wise multiplication two real vectors.
 *
 * @param [in]     v1        Pointer to first operand \ref ifx_Vector_R_t
 *                           on which element wise multiplication is applied to.
 * @param [in]     v2        Pointer to first operand \ref ifx_Vector_R_t
 *                           on which element wise multiplication is applied to.
 * @param [out]    result    Pointer to result vector \ref ifx_Vector_R_t
 *                           containing the results of element wise multiplication.
 */
IFX_DLL_PUBLIC
void ifx_vec_mul_r(const ifx_Vector_R_t* v1,
                   const ifx_Vector_R_t* v2,
                   ifx_Vector_R_t* result);

/**
 * @brief Computes element wise multiplication two complex vectors.
 *
 * @param [in]     v1        Pointer to first operand \ref ifx_Vector_C_t
 *                           on which element wise multiplication is applied to.
 * @param [in]     v2        Pointer to first operand \ref ifx_Vector_C_t
 *                           on which element wise multiplication is applied to.
 * @param [out]    result    Pointer to result vector \ref ifx_Vector_C_t
 *                           containing the results of element wise multiplication.
 */
IFX_DLL_PUBLIC
void ifx_vec_mul_c(const ifx_Vector_C_t* v1,
                   const ifx_Vector_C_t* v2,
                   ifx_Vector_C_t* result);

/**
 * @brief Computes element wise multiplication two (complex & real) vectors.
 *
 * @param [in]     v1        Pointer to first operand \ref ifx_Vector_C_t
 *                           on which element wise multiplication is applied to.
 * @param [in]     v2        Pointer to first operand \ref ifx_Vector_R_t
 *                           on which element wise multiplication is applied to.
 * @param [out]    result    Pointer to result vector \ref ifx_Vector_C_t
 *                           containing the results of element wise multiplication.
 */
IFX_DLL_PUBLIC
void ifx_vec_mul_cr(const ifx_Vector_C_t* v1,
                    const ifx_Vector_R_t* v2,
                    ifx_Vector_C_t* result);

/**
 * @brief Computes absolute array (modulus) from Real input data. Absolute = |data|
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           on which Absolute operation is to be performed.
 * @param [out]    output    Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           containing the absolute values.
 */
IFX_DLL_PUBLIC
void ifx_vec_abs_r(const ifx_Vector_R_t* input,
                   ifx_Vector_R_t* output);
/**
 * @brief Computes absolute array from Complex input data. Absolute = sqrt(real^2 + imag^2)
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_C_t
 *                           on which Absolute operation is to be performed.
 * @param [out]    output    Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           containing the absolute values.
 */
IFX_DLL_PUBLIC
void ifx_vec_abs_c(const ifx_Vector_C_t* input,
                   ifx_Vector_R_t* output);

/**
 * @brief Removes a scalar value from each sample of a real vector.
 *
 * @param [in]     input               Pointer to data memory defined by \ref ifx_Vector_R_t
 *                                     from which the given scalar is to be subtracted.
 * @param [in]     scalar_value        Floating point scalar.
 * @param [out]    output              Pointer to data memory containing the result of scalar subtraction.
 *                                     In-place operation may be achieved by passing the same pointer for
 *                                     both input and output.
 */
IFX_DLL_PUBLIC
void ifx_vec_sub_rs(const ifx_Vector_R_t* input,
                    ifx_Float_t scalar_value,
                    ifx_Vector_R_t* output);

/**
 * @brief Removes a scalar value from each sample of a complex vector.
 *
 * @param [in]     input               Pointer to data memory defined by \ref ifx_Vector_C_t
 *                                     from which the given scalar is to be subtracted.
 * @param [in]     scalar_value        Floating point scalar.
 * @param [out]    output              Pointer to data memory containing the result of scalar subtraction.
 *                                     In-place operation may be achieved by passing the same pointer for
 *                                     both input and output.
 */
IFX_DLL_PUBLIC
void ifx_vec_sub_cs(const ifx_Vector_C_t* input,
                    ifx_Complex_t scalar_value,
                    ifx_Vector_C_t* output);

/**
 * @brief Computes multiplication of a given real vector by a scalar.
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_R_t.
 * @param [in]     scale     Value by which each element in input array gets multiplied.
 * @param [out]    output    Pointer to the result vector \ref ifx_Vector_R_t.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_scale_r(const ifx_Vector_R_t* input,
                     ifx_Float_t scale,
                     ifx_Vector_R_t* output);

/**
 * @brief Applies linear complex value scaling to a real vector.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Real vector on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Complex floating point values of scalar (a in above equation).
 * @param [out]    output    Complex vector where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input vector. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_scale_rc(const ifx_Vector_R_t* input,
                      ifx_Complex_t scale,
                      ifx_Vector_C_t* output);

/**
 * @brief Computes multiplication of a given complex vector by a scalar.
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Vector_C_t.
 * @param [in]     scale     Value by which each element in input array gets multiplied.
 * @param [out]    output    Pointer to the result vector \ref ifx_Vector_C_t.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_scale_c(const ifx_Vector_C_t* input,
                     ifx_Complex_t scale,
                     ifx_Vector_C_t* output);

/**
 * @brief Applies linear real value scaling to a complex vector.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Complex vector on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Real floating point values of scalar (a in above equation).
 * @param [out]    output    Complex vector where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input vector. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_scale_cr(const ifx_Vector_C_t* input,
                      ifx_Float_t scale,
                      ifx_Vector_C_t* output);

/**
 * @brief Computes the euclidean distance between two n-D vectors v1 and v2 (both being of the same size).
 *
 * @param [in]     v1        Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           containing initial points for euclidean distance calculation.
 * @param [in]     v2        Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           containing terminal points for euclidean distance calculation.
 *
 * @return Euclidean distance between two input vectors v1 and v2
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_distance_r(const ifx_Vector_R_t* v1,
                               const ifx_Vector_R_t* v2);

/**
 * @brief Sorts real vector indices.
 *
 * @param [in]     input               Pointer to data memory defined by \ref ifx_Vector_R_t.
 * @param [in]     order               Sorting order defined by \ref ifx_Vector_Sort_Order_t.
 * @param [out]    sorted_idxs         Pointer to sorted indices array.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_isort_r(const ifx_Vector_R_t* input,
                     ifx_Vector_Sort_Order_t order,
                     uint32_t* sorted_idxs);

/**
 * @brief Applies multiply accumulate (MAC) operation on real vectors.
 *
 * Math operation implemented by this method: result = a + b * scale
 *
 * @param [in]     v1        Real vector to be added i.e. "a" in above equation.
 * @param [in]     v2        Real vector to be scaled i.e. "b" in above equation.
 * @param [in]     scale     Real floating point values of scalar.
 * @param [out]    result    Real vector where the result of scaling will be stored.
 *                           It must be allocated and of the same dimensions as the given input vector. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_mac_r(const ifx_Vector_R_t* v1,
                   const ifx_Vector_R_t* v2,
                   const ifx_Float_t scale,
                   ifx_Vector_R_t* result);

/**
 * @brief Applies multiply accumulate (MAC) operation on complex vectors.
 *
 * Math operation implemented by this method: result = a + b * scale
 *
 * @param [in]     v1        Complex vector to be added i.e. "a" in above equation
 * @param [in]     v2        Complex vector to be scaled i.e. "b" in above equation
 * @param [in]     scale     Complex floating point values of scalar
 *
 * @param [out]    result    Complex vector where the result of scaling will be stored.
 *                           It must be allocated and of the same dimensions as the given input vector. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_mac_c(const ifx_Vector_C_t* v1,
                   const ifx_Vector_C_t* v2,
                   const ifx_Complex_t scale,
                   ifx_Vector_C_t* result);

/**
 * @brief Computes the arithmetic mean of a given real vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_R_t
 *                           from which mean is calculated.
 *
 * @return Mean value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_mean_r(const ifx_Vector_R_t* vector);

/**
 * @brief Computes the arithmetic mean of a given complex vector.
 *
 * @param [in]     vector    Pointer to the memory containing array defined by \ref ifx_Vector_C_t
 *                           from which mean is calculated.
 *
 * @return Mean value of the passed input vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_vec_mean_c(const ifx_Vector_C_t* vector);

/**
 * @brief Returns the maximum value of a real vector.
 *
 * @param [in]     vector    Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           from which max value is extracted.
 *
 * @return Maximum value of the passed input real vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_max_r(const ifx_Vector_R_t* vector);

/**
 * @brief Computes the variance of a real vector.
 *
 * @param [in]     vector    Pointer to data memory defined by \ref ifx_Vector_R_t
 *                           from which variance is calculated.
 *
 * @return Variance value of the passed input real vector.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_vec_var_r(const ifx_Vector_R_t* vector);

/**
 * @brief Finds local maxima with at least threshold value,
 *        and returns the indices at which the peaks occur.
 *
 * [peaks,idx] = findpeaks(vector,'MinPeakHeight',threshold);
 *
 * @param [in]     vector              Pointer to data memory defined by \ref ifx_Vector_R_t.
 * @param [in]     threshold           Min Peak Height Threshold.
 * @param [in]     num_maxima          Max number of local maxima to be identified.
 * @param [out]    maxima_idxs         Pointer to local maxima indices array.
 *
 * @return Number of identified local maxima of the passed input real vector.
 *
 */
IFX_DLL_PUBLIC
uint32_t ifx_vec_local_maxima(const ifx_Vector_R_t* vector,
                              ifx_Float_t threshold,
                              uint32_t num_maxima,
                              uint32_t* maxima_idxs);


/**
 * @brief Clears all elements of real vector defined by \ref ifx_Vector_R_t.
 *
 * @param [in]     vector    Pointer to real vector to be cleared.
 * 
 */
IFX_DLL_PUBLIC
void ifx_vec_clear_r(ifx_Vector_R_t* vector);

/**
 * @brief Clears all elements of complex vector defined by \ref ifx_Vector_C_t.
 *
 * @param [in]     vector    Pointer to complex vector to be cleared.
 * 
 */
IFX_DLL_PUBLIC
void ifx_vec_clear_c(ifx_Vector_C_t* vector);

/**
 * @brief Populates vector with evenly spaced numbers over a specified interval.
 * Uses inputs 'start','end' and the length of the 'output' vector to populate
 * it with equally spaced values. The starting value is 'start' and the last value is
 * 'end - delta'. The increment/decrement 'delta' is derived as
 * \f[
 * \mathrm{delta} = \frac{(\mathrm{end-start})}{length(\mathrm{output})}
 * \f]
 *
 * @param [in]     start    starting value of linear space
 *
 * @param [in]     end      limiting value of linear space
 *
 * @param [out]    output   Pointer to output vector defined by \ref ifx_Vector_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_vec_linspace_r(const ifx_Float_t start,
    const ifx_Float_t end,
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

#endif /* IFX_BASE_VECTOR_H */
