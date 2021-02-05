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
 * @file Matrix.h
 *
 * \brief \copybrief gr_matrix
 *
 * For details refer to \ref gr_matrix
 */

#ifndef IFX_BASE_MATRIX_H
#define IFX_BASE_MATRIX_H

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
#define IFX_MAT_DAT(m)           ((m)->d)
#define IFX_MAT_ROWS(m)          ((m)->rows)
#define IFX_MAT_COLS(m)          ((m)->cols)
#define IFX_MAT_LDA(m)           ((m)->lda)
#define IFX_MAT_SIZE(m)          ((size_t)IFX_MAT_COLS(m) * (size_t)IFX_MAT_ROWS(m))
#define IFX_MAT_OFFSET(m, r, c)  ((size_t)IFX_MAT_LDA(m) * (size_t)(r) + (size_t)(c))
#define IFX_MAT_AT(m, r, c)      (IFX_MAT_DAT(m)[IFX_MAT_OFFSET(m, r, c)])

// Condition check macro adaptations for Matrix module -----------------------
#define IFX_MAT_BRK_DIM(m1, m2)         IFX_ERR_BRK_COND((mCols(m1) != mCols(m2)) || (mRows(m1) != mRows(m2)), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_DIM(m1, m2, v)      IFX_ERR_BRV_COND((mCols(m1) != mCols(m2)) || (mRows(m1) != mRows(m2)), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_SIZE(m1, m2)        IFX_ERR_BRK_COND(mSize(m1) != mSize(m2), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_SIZE(m1, m2, v)     IFX_ERR_BRV_COND(mSize(m1) != mSize(m2), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_SQUARE(m)           IFX_ERR_BRK_COND(mRows(m) != mCols(m), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_SQUARE(m, v)        IFX_ERR_BRV_COND(mRows(m) != mCols(m), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_DIM_COL_ROW(m1, m2)     IFX_ERR_BRK_COND(mCols(m1) != mRows(m2), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_DIM_COL_ROW(m1, m2, v)  IFX_ERR_BRV_COND(mCols(m1) != mRows(m2), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_DIM_COL(m1, m2)     IFX_ERR_BRK_COND(mCols(m1) != mCols(m2), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_DIM_COL(m1, m2, v)  IFX_ERR_BRV_COND(mCols(m1) != mCols(m2), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_DIM_ROW(m1, m2)     IFX_ERR_BRK_COND(mRows(m1) != mRows(m2), IFX_ERROR_DIMENSION_MISMATCH)
#define IFX_MAT_BRV_DIM_ROW(m1, m2, v)  IFX_ERR_BRV_COND(mRows(m1) != mRows(m2), IFX_ERROR_DIMENSION_MISMATCH, v)

#define IFX_MAT_BRK_IDX(m, r, c)        IFX_ERR_BRK_COND((r >= mRows(m)) || (c >= mCols(m)), IFX_ERROR_INDEX_OUT_OF_BOUNDS)
#define IFX_MAT_BRV_IDX(m, r, c, v)     IFX_ERR_BRV_COND((r >= mRows(m)) || (c >= mCols(m)), IFX_ERROR_INDEX_OUT_OF_BOUNDS, v)

#define IFX_MAT_BRK_ROWS(m, r)          IFX_ERR_BRK_COND(r > mRows(m), IFX_ERROR_INDEX_OUT_OF_BOUNDS)
#define IFX_MAT_BRV_ROWS(m, r, v)       IFX_ERR_BRK_COND(r > mRows(m), IFX_ERROR_INDEX_OUT_OF_BOUNDS, v)

#define IFX_MAT_BRK_COLS(m, c)          IFX_ERR_BRK_COND(c > mCols(m), IFX_ERROR_INDEX_OUT_OF_BOUNDS)
#define IFX_MAT_BRV_COLS(m, c, v)       IFX_ERR_BRK_COND(c > mCols(m), IFX_ERROR_INDEX_OUT_OF_BOUNDS, v)

#define IFX_MAT_BRK_VALID(m)  do {               \
        IFX_ERR_BRK_NULL(m);                     \
        IFX_ERR_BRK_ARGUMENT(mDat(m) == NULL)    \
    } while(0)
#define IFX_MAT_BRV_VALID(m, r)  do {            \
        IFX_ERR_BRV_NULL(m, r);                  \
        IFX_ERR_BRV_ARGUMENT(mDat(m) == NULL, r) \
    } while(0)

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @brief Defines the structure for a two-dimensional floating point data array.
 *        Use type ifx_Matrix_R_t for this struct.
 *        Data length is fixed in this matrix i.e. matrix neither grows nor shrinks.
 *        The data is arranged sequentially in a row-major order (C order), i.e., all elements of a given row are
 *        placed in successive memory locations, as depicted in the illustrations.
 *
 * @image html img_matrix_memory_map_wiki.png "Illustration of row-major order" width=400px
 *
 * @image html img_matrix_memory_map_wiki_c.png "Illustration of accessing a matrix in row-major order in C (starting from index 0)" width=400px
 *
 * The above 2 images have been sourced from <a href="https://en.wikipedia.org/wiki/Row-_and_column-major_order">here</a>.
 *
 * @image html img_matrix_memory_map.png "Illustration showing memory arrangement of matrix data in ifx_Matrix_R_t " width=600px
 *
 */
struct ifx_Matrix_R_s
{
    ifx_Float_t* d;          /**< Pointer to floating point memory containing data values */
    uint32_t     rows;       /**< Number of rows in the matrix */
    uint32_t     cols;       /**< Number of columns in the matrix */
    uint32_t     lda : 31;   /**< Number of sequential memory locations to jump for the next row */
    uint8_t      owns_d : 1; /**< Set to 1 if the matrix owns its data and has to free it */
};

/**
 * @brief Defines the structure for a two-dimensional Complex data array.
 *        Use type ifx_Matrix_C_t for this struct.
 *        Data length is fixed in this matrix i.e. matrix neither grows nor shrinks.
 *        The data is arranged sequentially in a row-major order, i.e., all elements of a given row are
 *        placed in successive memory locations, as depicted in the illustrations.
 *
 * @image html img_matrix_memory_map_wiki.png "Illustration of row-major order" width=400px
 *
 * @image html img_matrix_memory_map_wiki_c.png "Illustration of accessing a matrix in row-major order in C (starting from index 0)" width=400px
 *
 * The above 2 images have been sourced from <a href="https://en.wikipedia.org/wiki/Row-_and_column-major_order">here</a>.
 *
 * @image html img_matrix_memory_map.png "Illustration showing memory arrangement of matrix data in ifx_Matrix_C_t" width=600px
 *
 */
struct ifx_Matrix_C_s
{
    ifx_Complex_t* d;          /**< Pointer to floating point memory containing data values */
    uint32_t       rows;       /**< Number of rows in the matrix */
    uint32_t       cols;       /**< Number of columns in the matrix */
    uint32_t       lda : 31;   /**< Number of sequential memory locations to jump for the next row */
    uint8_t        owns_d : 1; /**< Set to 1 if the matrix owns its data and has to free it */
};

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Math_Elements
  * @{
  */

/** @defgroup gr_matrix Matrix
  * @brief API for operations on Matrix data structures
  *
  * Supports matrix operations such as creation, destruction
  *        and mathematical manipulations.
  *
  * @{
  */

/**
 * @brief Initializes a real matrix \ref ifx_Matrix_R_t with a data element of
 *        specified size, to prevent memory allocation on the heap.
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     d         Data pointer to assign the matrix
 * @param [in]     rows      Number of rows
 * @param [in]     columns   Number of columns
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_init_r(ifx_Matrix_R_t* matrix,
                    ifx_Float_t* d,
                    const uint32_t rows,
                    const uint32_t columns);

/**
 * @brief Initializes a complex matrix \ref ifx_Matrix_C_t with a data element of
 *        specified size, to prevent memory allocation on the heap.
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     d         Data pointer to assign the matrix
 * @param [in]     rows      Number of rows
 * @param [in]     columns   Number of columns
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_init_c(ifx_Matrix_C_t* matrix,
                    ifx_Complex_t* d,
                    const uint32_t rows,
                    const uint32_t columns);

/**
 * @brief ...
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     d         Data pointer to assign the matrix
 * @param [in]     rows      Number of row
 * @param [in]     columns   Number of columns
 * @param [in]     lda       Number of sequential memory locations to jump for the next row
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_rawview_r(ifx_Matrix_R_t* matrix,
                       ifx_Float_t* d,
                       const uint32_t rows,
                       const uint32_t columns,
                       const uint32_t lda);

/**
 * @brief ...
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     d         Data pointer to assign the matrix
 * @param [in]     rows      Number of rows
 * @param [in]     columns   Number of columns
 * @param [in]     lda       Number of sequential memory locations to jump for the next row
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_rawview_c(ifx_Matrix_C_t* matrix,
                       ifx_Complex_t* d,
                       const uint32_t rows,
                       const uint32_t columns,
                       const uint32_t lda);

/**
 * @brief ...
 *
 * @param [in]     matrix              Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     source              Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     row_offset          Row offset
 * @param [in]     column_offset       Column offset
 * @param [in]     rows                Number of rows to view
 * @param [in]     columns             Number of columns to view
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_view_r(ifx_Matrix_R_t* matrix,
                    ifx_Matrix_R_t* source,
                    const uint32_t row_offset,
                    const uint32_t column_offset,
                    const uint32_t rows,
                    const uint32_t columns);

/**
 * @brief ...
 *
 * @param [in]     matrix              Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     source              Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     row_offset          Row offset
 * @param [in]     column_offset       Column offset
 * @param [in]     rows                Number of rows to view
 * @param [in]     columns             Number of columns to view
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_view_c(ifx_Matrix_C_t* matrix,
                    ifx_Matrix_C_t* source,
                    const uint32_t row_offset,
                    const uint32_t column_offset,
                    const uint32_t rows,
                    const uint32_t columns);

/**
 * @brief ...
 *
 * @param [in]     matrix              Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     source              Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     row_offset          Row offset
 * @param [in]     rows                Number of rows to view
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_view_rows_r(ifx_Matrix_R_t* matrix,
                         ifx_Matrix_R_t* source,
                         const uint32_t row_offset,
                         const uint32_t rows);

/**
 * @brief ...
 *
 * @param [in]     matrix              Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     source              Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     row_offset          Row offset
 * @param [in]     rows                Number of rows to view
 */
IFX_DLL_PUBLIC
void ifx_mat_view_rows_c(ifx_Matrix_C_t* matrix,
                         ifx_Matrix_C_t* source,
                         const uint32_t row_offset,
                         const uint32_t rows);

/**
 * @brief Allocates memory for a real matrix with a specified number of
 *        rows and columns and initializes it to zero.
 *        The data is arranged sequentially row-wise, i.e., all elements of a given row are
 *        placed in successive memory locations, as depicted in the illustration.
 *
 * @image html img_matrix_memory_map.png "Illustration showing memory arrangement of matrix data in ifx_Matrix_R_t" width=600px
 *
 * see \ref ifx_Matrix_R_t for more details.
 *
 * @param [in]     rows      Number of rows
 * @param [in]     columns   Number of columns
 *
 * @return Pointer to allocated and initialized real matrix structure or NULL if allocation failed.
 *
 */
IFX_DLL_PUBLIC
ifx_Matrix_R_t* ifx_mat_create_r(const uint32_t rows,
                                 const uint32_t columns);

/**
 * @brief Allocates memory for a complex matrix with a specified number of
 *        rows and columns and initializes it to zero.
 *        The data is arranged sequentially row-wise, i.e., all elements of a given row are
 *        placed in successive memory locations, as depicted in the illustration.
 *
 * @image html img_matrix_memory_map.png "Illustration showing memory arrangement of matrix data in ifx_Matrix_C_t" width=600px
 *
 * see \ref ifx_Matrix_C_t for more details.
 *
 * @param [in]     rows      Number of rows
 * @param [in]     columns   Number of columns
 *
 * @return Pointer to allocated and initialized complex matrix structure or NULL if allocation failed.
 *
 */
IFX_DLL_PUBLIC
ifx_Matrix_C_t* ifx_mat_create_c(const uint32_t rows,
                                 const uint32_t columns);


/**
 * @brief De-initializes a real matrix \ref ifx_Matrix_R_t
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_deinit_r(ifx_Matrix_R_t* matrix);

/**
 * @brief De-initializes a complex matrix \ref ifx_Matrix_C_t
 *
 * @param [in]     matrix    Pointer to data memory defined by \ref ifx_Matrix_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_deinit_c(ifx_Matrix_C_t* matrix);

/**
 * @brief Frees memory for a real matrix defined by \ref ifx_mat_create_r
 *        and sets the length of the matrix to zero.
 *
 * @param [in,out] matrix    Pointer to an allocated matrix instance defined
 *                           by \ref ifx_Matrix_R_t
 */
IFX_DLL_PUBLIC
void ifx_mat_destroy_r(ifx_Matrix_R_t* matrix);

/**
 * @brief Frees memory for a complex matrix defined by \ref ifx_mat_create_c
 *        and sets the length of the matrix to zero.
 *
 * @param [in,out] matrix    Pointer to an allocated matrix instance defined
 *                           by \ref ifx_Matrix_C_t
 */
IFX_DLL_PUBLIC
void ifx_mat_destroy_c(ifx_Matrix_C_t* matrix);

/**
 * @brief Blits elements of a given real matrix, to a new created real matrix
 *        with user defined dimensions.
 *
 * @param [in]     from                Pointer to data memory defined by \ref ifx_Matrix_R_t
 * @param [in]     from_row            Row location from where to blit
 * @param [in]     num_rows            Number of rows to blit
 * @param [in]     from_column         Column location from where to blit
 * @param [in]     num_columns         Number of columns to blit
 * @param [out]    to                  Pointer to data memory defined by \ref ifx_Matrix_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_blit_r(const ifx_Matrix_R_t* from,
                    uint32_t from_row,
                    uint32_t num_rows,
                    uint32_t from_column,
                    uint32_t num_columns,
                    ifx_Matrix_R_t* to);

/**
 * @brief Blits elements of a given complex matrix, to a new created complex matrix
 *        with user defined dimensions.
 *
 * @param [in]     from                Pointer to data memory defined by \ref ifx_Matrix_C_t
 * @param [in]     from_row            Row location from where to blit
 * @param [in]     num_rows            Number of rows to blit
 * @param [in]     from_column         Column location from where to blit
 * @param [in]     num_columns         Number of columns to blit
 * @param [out]    to                  Pointer to data memory defined by \ref ifx_Matrix_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_blit_c(const ifx_Matrix_C_t* from,
                    uint32_t from_row,
                    uint32_t num_rows,
                    uint32_t from_column,
                    uint32_t num_columns,
                    ifx_Matrix_C_t* to);
/**
 * @brief Copies the elements from a real source matrix to a real destination matrix.
 *        The condition is that both source and destination matrix instance should be allocated
 *        and populated (pointers inside the matrix instance should not be null) using the
 *        \ref ifx_mat_create_r. No memory allocation is done in this function.
 *
 * @param [in]     from      Pointer to an allocated and populated source
 *                           matrix instance defined by \ref ifx_Matrix_R_t
 * @param [in,out] to        Pointer to an allocated and populated, yet empty
 *                           (data values of the matrix are still zero) destination
 *                           matrix instance defined by \ref ifx_Matrix_R_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_copy_r(const ifx_Matrix_R_t* from,
                    ifx_Matrix_R_t* to);

/**
 * @brief Copies the elements from a complex source matrix to a complex destination matrix.
 *        The condition is that both source and destination matrix instance should be allocated
 *        and populated (pointers inside the matrix instance should not be null) using the
 *        \ref ifx_mat_create_c. No memory allocation is done in this function.
 *
 * @param [in]     from      Pointer to an allocated and populated source
 *                           matrix instance defined by \ref ifx_Matrix_C_t
 * @param [in,out] to        Pointer to an allocated and populated, yet empty
 *                           (data values of the matrix are still zero) destination
 *                           matrix instance defined by \ref ifx_Matrix_C_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_copy_c(const ifx_Matrix_C_t* from,
                    ifx_Matrix_C_t* to);

/**
 * @brief Sets a user defined value at a given row, column location in a real matrix.
 *
 * @param [in,out] matrix    Pointer to an allocated and populated real valued
 *                           matrix instance defined by \ref ifx_Matrix_R_t
 * @param [in]     row       Row location where the value is to be set
 * @param [in]     column    Column location where the value is to be set
 * @param [in]     value     User defined value defined by \ref ifx_Float_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_element_r(ifx_Matrix_R_t* matrix,
                           uint32_t row,
                           uint32_t column,
                           ifx_Float_t value);

/**
 * @brief Sets a user defined value at a given row, column location in a complex matrix.
 *
 * @param [in,out] matrix    Pointer to an allocated and populated complex valued
 *                           matrix instance defined by \ref ifx_Matrix_C_t
 * @param [in]     row       Row location where the value is to be set
 * @param [in]     column    Column location where the value is to be set
 * @param [in]     value     User defined value defined by \ref ifx_Complex_t
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_element_c(ifx_Matrix_C_t* matrix,
                           uint32_t row,
                           uint32_t column,
                           ifx_Complex_t value);

/**
 * @brief Returns a user defined value set at a given row,column location in a real matrix.
 *
 * @param [in]     matrix    Pointer to an allocated and populated real valued
 *                           matrix instance defined by \ref ifx_Matrix_R_t
 * @param [in]     row       Row location where the value can be obtained
 * @param [in]     column    Column location where the value can be obtained
 *
 * @return Value at the specified index.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_get_element_r(const ifx_Matrix_R_t* matrix,
                                  uint32_t row,
                                  uint32_t column);

/**
 * @brief Returns a user defined value set at a given row, column location in a complex matrix.
 *
 * @param [in]     matrix    Pointer to an allocated and populated complex valued
 *                           matrix instance defined by \ref ifx_Matrix_C_t
 * @param [in]     row       Row location where the value can be obtained
 * @param [in]     column    Column location where the value can be obtained
 *
 * @return Value at the specified index.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_mat_get_element_c(const ifx_Matrix_C_t* matrix,
                                    uint32_t row,
                                    uint32_t column);

/**
 * @brief Copies a user defined sequence of real values to a user defined row index in a real matrix.
 *        The count of the input real values should not be greater than the number of columns in the matrix.
 *
 * @param [in,out] matrix              Pointer to an allocated and populated real valued
 *                                     matrix instance defined by \ref ifx_Matrix_R_t
 * @param [in]     row_index           Row number that is to be filled by the user defined vector
 * @param [in]     row_values          Pointer to user defined vector defined by \ref ifx_Float_t
 * @param [in]     count               Number of elements in the user defined vector
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_row_r(ifx_Matrix_R_t* matrix,
                       uint32_t row_index,
                       const ifx_Float_t* row_values,
                       uint32_t count);

/**
 * @brief Copies a user defined sequence of complex numbers values to a user defined row index in a complex matrix.
 *        The count of the input complex numbers should not be greater than the number of columns in the matrix.
 *
 * @param [in,out] matrix              Pointer to an allocated and populated complex valued
 *                                     matrix instance defined by \ref ifx_Matrix_C_t
 * @param [in]     row_index           Row number that is to be filled by the user defined vector
 * @param [in]     row_values          Pointer to User defined vector defined by \ref ifx_Complex_t
 * @param [in]     count               Number of elements in the user defined vector
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_row_c(ifx_Matrix_C_t* matrix,
                       uint32_t row_index,
                       const ifx_Complex_t* row_values,
                       uint32_t count);

/**
 * @brief Copies a user defined real valued vector \ref ifx_Vector_R_t to a user defined row index in a real matrix.
 *        The length of the input real valued matrix should not be greater than the number of columns in the matrix.
 *
 * @param [in,out] matrix              Pointer to an allocated and populated real valued
 *                                     matrix instance defined by \ref ifx_Matrix_R_t
 * @param [in]     row_index           Row number that is to be filled by the user defined vector
 * @param [in]     row_values          Pointer to vector from which the data is to be copied to the specified row
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_row_vector_r(ifx_Matrix_R_t* matrix,
                              uint32_t row_index,
                              const ifx_Vector_R_t* row_values);

/**
 * @brief Copies a user defined complex valued matrix \ref ifx_Vector_C_t to a user defined row index in a complex matrix.
 *        The length of the input complex valued matrix should not be greater than the number of columns in the matrix.
 *
 * @param [in,out] matrix              Pointer to an allocated and populated complex valued
 *                                     matrix instance defined by \ref ifx_Matrix_C_t
 * @param [in]     row_index           Row number that is to be filled by the user defined vector
 * @param [in]     row_values          Pointer to vector from which the data is to be copied to the specified row
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_set_row_vector_c(ifx_Matrix_C_t* matrix,
                              uint32_t row_index,
                              const ifx_Vector_C_t* row_values);

/**
 * @brief Returns a complex valued matrix pointing to defined row of the given complex matrix.
 *
 * @param [in]     matrix              The complex matrix, from which one row would be pointed to by the output vector
 * @param [in]     row_index           The row of the matrix the vector will point to.
 * @param [in]     row_view            Pointer to vector view to be initialized
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_get_rowview_c(const ifx_Matrix_C_t* matrix,
                           uint32_t row_index,
                           ifx_Vector_C_t* row_view);

/**
 * @brief Returns a real valued matrix pointing to defined row of the given real matrix.
 *
 * @param [in]     matrix              The real matrix, from which one row would be pointed to by the output vector
 * @param [in]     row_index           The row of the matrix the vector will point to.
 * @param [in]     row_view            Pointer to vector view to be initialized
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_get_rowview_r(const ifx_Matrix_R_t* matrix,
                           uint32_t row_index,
                           ifx_Vector_R_t* row_view);

/**
 * @brief Returns a real valued matrix pointing to defined column of the given real matrix.
 *
 * @param [in]     matrix              The real matrix, from which one row would be pointed to by the output vector.
 * @param [in]     col_index           The row of the matrix the vector will point to.
 * @param [in]     col_view            Pointer to vector view to be initialized
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_get_colview_r(const ifx_Matrix_R_t* matrix,
                           uint32_t col_index,
                           ifx_Vector_R_t* col_view);

/**
 * @brief Returns a complex valued matrix pointing to defined column of the given complex matrix.
 *
 * @param [in]     matrix              The complex matrix, from which one row would be pointed to by the output vector
 * @param [in]     col_index           The row of the matrix the vector will point to.
 * @param [in]     col_view            Pointer to vector view to be initialized
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_get_colview_c(const ifx_Matrix_C_t* matrix,
                           uint32_t col_index,
                           ifx_Vector_C_t* col_view);

/**
 * @brief Transposes a given real matrix and saves it in the given output matrix. This function doesn't support in-place
 *        transposing therefore it is not allowed input and output matrix pointing to the same matrix.
 *        This will result in IFX_ERROR_IN_PLACE_CALCULATION_NOT_SUPPORTED.
 *
 * @param [in]     matrix              Real matrix to transpose.
 * @param [out]    transposed          Real matrix the transposed matrix will be stored in must be allocated
 *                                     and of the same size as the given input matrix.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_transpose_r(const ifx_Matrix_R_t* matrix,
                         ifx_Matrix_R_t* transposed);

/**
 * @brief Transposes a given complex matrix and saves it in the given output matrix. This function doesn't support
 *        in-place transposing therefore it is not allowed input and output matrix pointing to the same matrix.
 *        This will result in IFX_ERROR_IN_PLACE_CALCULATION_NOT_SUPPORTED.

 * @param [in]     matrix              Complex matrix to transpose.
 * @param [out]    transposed          Complex matrix the transposed matrix will be stored in must be allocated
 *                                     and of the same size as the given input matrix.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_transpose_c(const ifx_Matrix_C_t* matrix,
                         ifx_Matrix_C_t* transposed);

/**
 * @brief Adds two real matrices and saves the result of addition into given output matrix.
 *
 * Math operation implemented by this method: Z = X + Y
 *
 * @param [in]     matrix_l  Real matrix as left argument (X in above equation).
 * @param [in]     matrix_r  Real matrix as right argument (Y in above equation).
 * @param [out]    result    Real matrix where the result of addition will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_add_r(const ifx_Matrix_R_t* matrix_l,
                   const ifx_Matrix_R_t* matrix_r,
                   ifx_Matrix_R_t* result);

IFX_DLL_PUBLIC
void ifx_mat_add_rs(const ifx_Matrix_R_t* input,
                    const ifx_Float_t scalar,
                    ifx_Matrix_R_t* output);

/**
 * @brief Adds two complex matrices and saves the result of addition into given output matrix.
 *
 * Math operation implemented by this method: Z = X + Y
 *
 * @param [in]     matrix_l  Complex matrix as left argument (X in above equation).
 * @param [in]     matrix_r  Complex matrix as right argument (Y in above equation).
 * @param [out]    result    Complex matrix where the result of addition will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_add_c(const ifx_Matrix_C_t* matrix_l,
                   const ifx_Matrix_C_t* matrix_r,
                   ifx_Matrix_C_t* result);


IFX_DLL_PUBLIC
void ifx_mat_add_cs(const ifx_Matrix_C_t* input,
                    const ifx_Complex_t scalar,
                    ifx_Matrix_C_t* output);

/**
 * @brief Subtracts two real matrices and saves the result of subtraction into given output matrix.
 *
 * Math operation implemented by this method: Z = X - Y
 *
 * @param [in]     matrix_l  Real matrix as left argument (X in above equation).
 * @param [in]     matrix_r  Real matrix as right argument (Y in above equation).
 * @param [out]    result    Real matrix where the result of subtraction will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_sub_r(const ifx_Matrix_R_t* matrix_l,
                   const ifx_Matrix_R_t* matrix_r,
                   ifx_Matrix_R_t* result);

IFX_DLL_PUBLIC
void ifx_mat_sub_rs(const ifx_Matrix_R_t* input,
                    const ifx_Float_t scalar,
                    ifx_Matrix_R_t* output);

/**
 * @brief Subtracts two complex matrices and saves the result of subtraction into given output matrix.
 *
 * Math operation implemented by this method: Z = X - Y
 *
 * @param [in]     matrix_l  Complex matrix as left argument (X in above equation).
 * @param [in]     matrix_r  Complex matrix as right argument (Y in above equation).
 * @param [out]    result    Complex matrix where the result of subtraction will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_sub_c(const ifx_Matrix_C_t* matrix_l,
                   const ifx_Matrix_C_t* matrix_r,
                   ifx_Matrix_C_t* result);

IFX_DLL_PUBLIC
void ifx_mat_sub_cs(const ifx_Matrix_C_t* input,
                    const ifx_Complex_t scalar,
                    ifx_Matrix_C_t* output);

/**
 * @brief Applies linear real value scaling to a real matrix.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Real matrix on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Real floating point values of scalar (a in above equation).
 * @param [out]    output    Real matrix where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_scale_r(const ifx_Matrix_R_t* input,
                     const ifx_Float_t scale,
                     ifx_Matrix_R_t* output);

/**
 * @brief Applies linear complex value scaling to a real matrix.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Real matrix on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Complex floating point values of scalar (a in above equation).
 * @param [out]    output    Complex matrix where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_scale_rc(const ifx_Matrix_R_t* input,
                      const ifx_Complex_t scale,
                      ifx_Matrix_C_t* output);

/**
 * @brief Applies linear complex value scaling to a complex matrix.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Complex matrix on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Complex floating point values of scalar (a in above equation).
 * @param [out]    output    Complex matrix where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_scale_c(const ifx_Matrix_C_t* input,
                     const ifx_Complex_t scale,
                     ifx_Matrix_C_t* output);

/**
 * @brief Applies linear real value scaling to a complex matrix.
 *
 * Math operation implemented by this method: Z = a * X
 *
 * @param [in]     input     Complex matrix on which scaling is to be applied (X in above equation).
 * @param [in]     scale     Real floating point values of scalar (a in above equation).
 * @param [out]    output    Complex matrix where the result of scaling will be stored (Z in above equation).
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_scale_cr(const ifx_Matrix_C_t* input,
                      const ifx_Float_t scale,
                      ifx_Matrix_C_t* output);

/**
 * @brief Applies multiply accumulate (MAC) operation on real matrices.
 *
 * Math operation implemented by this method: output = a + b * scale
 *
 * @param [in]     m1        Real matrix to be added i.e. "a" in above equation
 * @param [in]     m2        Real matrix to be scaled i.e. "b" in above equation
 * @param [in]     scale     Real floating point values of scalar
 * @param [out]    result    Real matrix where the result of scaling will be stored.
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mac_r(const ifx_Matrix_R_t* m1,
                   const ifx_Matrix_R_t* m2,
                   const ifx_Float_t scale,
                   ifx_Matrix_R_t* result);

/**
 * @brief Applies multiply accumulate (MAC) operation on complex matrices.
 *
 * Math operation implemented by this method: output = a + b * scale
 *
 * @param [in]     m1        Complex matrix to be added i.e. "a" in above equation
 * @param [in]     m2        Complex matrix to be scaled i.e. "b" in above equation
 * @param [in]     scale     Complex floating point values of scalar
 *
 * @param [out]    result    Complex matrix where the result of scaling will be stored.
 *                           It must be allocated and of the same dimensions as the given input matrix. Can be in-place.
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mac_c(const ifx_Matrix_C_t* m1,
                   const ifx_Matrix_C_t* m2,
                   const ifx_Complex_t scale,
                   ifx_Matrix_C_t* result);

/**
 * @brief Computes absolute matrix (modulus) from real input matrix. Absolute = |data|
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Matrix_R_t,
 *                           on which Absolute operation is to be performed.
 * @param [out]    output    Pointer to data memory defined by \ref ifx_Matrix_R_t,
 *                           containing the absolute values.
 */
IFX_DLL_PUBLIC
void ifx_mat_abs_r(const ifx_Matrix_R_t* input,
                   ifx_Matrix_R_t* output);

/**
 * @brief Computes absolute matrix (modulus) from complex input matrix. Absolute = |data|
 *
 * @param [in]     input     Pointer to data memory defined by \ref ifx_Matrix_C_t,
 *                           on which Absolute operation is to be performed.
 * @param [out]    output    Pointer to data memory defined by \ref ifx_Matrix_R_t,
 *                           containing the absolute values.
 */
IFX_DLL_PUBLIC
void ifx_mat_abs_c(const ifx_Matrix_C_t* input,
                   ifx_Matrix_R_t* output);

/**
 * @brief Computes the arithmetic sum of a real valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_R_t
 *                           to calculate the sum of all its elements.
 *
 * @return Sum of all elements' values of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_sum_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Computes the arithmetic sum from a complex valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_C_t
 *                           to calculate the sum of all its elements.
 *
 * @return Sum of all elements' values of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_mat_sum_c(const ifx_Matrix_C_t* matrix);

/**
 * @brief Computes the sum of squared values of a given real valued matrix.
 *
 * @param [in]     matrix    Pointer to the memory containing array defined by \ref ifx_Matrix_R_t
 *
 * @return Sum of squared values of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_sqsum_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Computes the sum of squared values of a given complex valued matrix.
 *
 * @param [in]     matrix    Pointer to the memory containing array defined by \ref ifx_Matrix_C_t
 *
 * @return Sum of squared values of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_sqsum_c(const ifx_Matrix_C_t* matrix);

/**
 * @brief Returns the biggest absolute value of a given real valued matrix.
 *
 * @param [in]     matrix    Pointer to the memory containing array defined by \ref ifx_Matrix_R_t.
 *
 * @return Maximum absolute value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_maxabs_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Returns the maximum absolute value of a given complex valued matrix.
 *
 * @param [in]     matrix    Pointer to the memory containing array defined by \ref ifx_Matrix_C_t.
 *
 * @return Maximum absolute value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_maxabs_c(const ifx_Matrix_C_t* matrix);

/**
 * @brief Computes the arithmetic mean from a real valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_R_t
 *                           from which mean is calculated.
 *
 * @return Mean real value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_mean_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Computes the arithmetic mean from a complex valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_C_t
 *                           from which mean is calculated.
 *
 * @return Mean complex value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Complex_t ifx_mat_mean_c(const ifx_Matrix_C_t* matrix);

/**
 * @brief Returns the maximum value of a real valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_R_t
 *                           from which max value is extracted.
 *
 * @return Maximum value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_max_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Computes the variance of a real valued matrix.
 *
 * @param [in]     matrix    Pointer to a data memory defined by \ref ifx_Matrix_R_t
 *                           from which variance is calculated.
 *
 * @return Variance value of the passed input matrix.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_mat_var_r(const ifx_Matrix_R_t* matrix);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA * inputB-Transpose
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_abt_r(const ifx_Matrix_R_t* inputA,
                   const ifx_Matrix_R_t* inputB,
                   ifx_Matrix_R_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA * inputB-conjugate-Transpose
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_abct_c(const ifx_Matrix_C_t* inputA,
                    const ifx_Matrix_C_t* inputB,
                    ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA * inputB-Transpose
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_abt_c(const ifx_Matrix_C_t* inputA,
                   const ifx_Matrix_C_t* inputB,
                   ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA * inputB-Transpose
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_abt_rc(const ifx_Matrix_R_t* inputA,
                    const ifx_Matrix_C_t* inputB,
                    ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA * inputB-Transpose
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_abt_cr(const ifx_Matrix_C_t* inputA,
                    const ifx_Matrix_R_t* inputB,
                    ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA-Transpose * inputB
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_atb_r(const ifx_Matrix_R_t* inputA,
                   const ifx_Matrix_R_t* inputB,
                   ifx_Matrix_R_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA-Transpose * inputB
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_atb_c(const ifx_Matrix_C_t* inputA,
                   const ifx_Matrix_C_t* inputB,
                   ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA-Transpose * inputB
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_atb_rc(const ifx_Matrix_R_t* inputA,
                    const ifx_Matrix_C_t* inputB,
                    ifx_Matrix_C_t* output);

/**
 * @brief Computes matrix multiplication for:
 *          output = inputA-Transpose * inputB
 *
 * @param [in]     inputA    ...
 * @param [in]     inputB    ...
 * @param [out]    output    ...
 */
IFX_DLL_PUBLIC
void ifx_mat_atb_cr(const ifx_Matrix_C_t* inputA,
                    const ifx_Matrix_R_t* inputB,
                    ifx_Matrix_C_t* output);
/**
 * @brief Compute the matrix-vector product for real-valued matrix and vector
 * @brief Computes the matrix-vector product for real-valued matrix and vector.
 *
 * Compute the matrix-vector product: result=matrix*vector
 *
 * For matrix having dimension MxN, vector must be of dimension N and result
 * must be of dimension M.
 *
 * @param [in]     matrix    matrix
 * @param [in]     vector    vector
 * @param [out]    result    resulting vector of matrix-vector multiplication
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_rv(const ifx_Matrix_R_t* matrix,
                    const ifx_Vector_R_t* vector,
                    ifx_Vector_R_t* result);

/**
 * @brief Computes the matrix-vector product for real-valued and transposed matrix and vector.
 *
 * Compute the matrix-vector product: result=(matrix)^T*vector
 *
 * For matrix having dimension MxN, vector must be of dimension M and result
 * must be of dimension N.
 *
 * Note that in contrast to \ref ifx_mat_mul_rv the matrix in the
 * multiplication is transposed.
 *
 * @param [in]     matrix    matrix
 * @param [in]     vector    vector
 * @param [out]    result    resulting vector of matrix-vector multiplication
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_trans_rv(const ifx_Matrix_R_t* matrix,
                          const ifx_Vector_R_t* vector,
                          ifx_Vector_R_t* result);

/**
 * @brief Computes the matrix-vector product for complex-valued matrix and vector.
 *
 * Compute the matrix-vector product: result=matrix*vector
 *
 * For matrix having dimension MxN, vector must be of dimension N and result
 * must be of dimension M.
 *
 * @param [in]     matrix    matrix
 * @param [in]     vector    vector
 * @param [out]    result    resulting vector of matrix-vector multiplication
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_cv(const ifx_Matrix_C_t* matrix,
                    const ifx_Vector_C_t* vector,
                    ifx_Vector_C_t* result);

/**
 * @brief Computes the matrix-vector product for complex-valued and transposed matrix and vector.
 *
 * Compute the matrix-vector product: result=(matrix)^T*vector
 *
 * For matrix having dimension MxN, vector must be of dimension M and result
 * must be of dimension N.
 *
 * Note that in contrast to \ref ifx_mat_mul_cv the matrix in the
 * multiplication is transposed.
 *
 * @param [in]     matrix    matrix
 * @param [in]     vector    vector
 * @param [out]    result    resulting vector of matrix-vector multiplication
 */
IFX_DLL_PUBLIC
void ifx_mat_trans_mul_cv(const ifx_Matrix_C_t* matrix,
                          const ifx_Vector_C_t* vector,
                          ifx_Vector_C_t* result);

/**
 * @brief Computes matrix product of two real matrices.
 *
 * Compute the matrix product matrix_l*matrix_r where matrix_l and matrix_r are
 * real matrices. The result is saved in result.
 *
 * The number of columns of matrix_l must match the number of rows of matrix_r.
 *
 * @param [in]     matrix_l  left matrix
 * @param [in]     matrix_r  right matrix
 * @param [in]     result    result, i.e., result=matrix_l*matrix_r
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_r(const ifx_Matrix_R_t* matrix_l,
                   const ifx_Matrix_R_t* matrix_r,
                   ifx_Matrix_R_t* result);

/**
 * @brief Computes matrix product of a real and a complex matrix.
 *
 * Compute the matrix product matrix_l*matrix_r where matrix_l is a real and
 * matrix_r is a complex matrix. The result is saved in result.
 *
 * The number of columns of matrix_l must match the number of rows of matrix_r.
 *
 * @param [in]     matrix_l  left matrix
 * @param [in]     matrix_r  right matrix
 * @param [in]     result    result, i.e., result=matrix_l*matrix_r
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_rc(const ifx_Matrix_R_t* matrix_l,
                    const ifx_Matrix_C_t* matrix_r,
                    ifx_Matrix_C_t* result);

/**
 * @brief Computes matrix product of two complex matrices.
 *
 * Compute the matrix product matrix_l*matrix_r where matrix_l and matrix_r are
 * complex matrices. The result is saved in result.
 *
 * The number of columns of matrix_l must match the number of rows of matrix_r.
 *
 * @param [in]     matrix_l  left matrix
 * @param [in]     matrix_r  right matrix
 * @param [in]     result    result, i.e., result=matrix_l*matrix_r
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_c(const ifx_Matrix_C_t* matrix_l,
                   const ifx_Matrix_C_t* matrix_r,
                   ifx_Matrix_C_t* result);

/**
 * @brief Computes matrix product of a complex and a real matrix.
 *
 * Compute the matrix product matrix_l*matrix_r where matrix_l is a complex and
 * matrix_r is a real matrix. The result is saved in result.
 *
 * The number of columns of matrix_l must match the number of rows of matrix_r.
 *
 * @param [in]     matrix_l  left matrix
 * @param [in]     matrix_r  right matrix
 * @param [in]     result    result, i.e., result=matrix_l*matrix_r
 *
 */
IFX_DLL_PUBLIC
void ifx_mat_mul_cr(const ifx_Matrix_C_t* matrix_l,
                    const ifx_Matrix_R_t* matrix_r,
                    ifx_Matrix_C_t* result);

/**
 * @brief Clears all elements of real matrix defined by \ref ifx_Matrix_R_t.
 *
 * Set all elements of the matrix to 0.
 *
 * @param [in]     matrix    Pointer to real matrix to be cleared.
 * 
 */
IFX_DLL_PUBLIC
void ifx_mat_clear_r(ifx_Matrix_R_t* matrix);

/**
 * @brief Clears all elements of complex matrix defined by \ref ifx_Matrix_C_t.
 *
 * Set all elements of the matrix to 0.
 *
 * @param [in]     matrix    Pointer to complex matrix to be cleared.
 * 
 */
IFX_DLL_PUBLIC
void ifx_mat_clear_c(ifx_Matrix_C_t* matrix);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_MATRIX_H */
