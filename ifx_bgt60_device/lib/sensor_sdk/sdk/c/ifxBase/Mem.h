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
* \file Mem.h
*
* \brief   \copybrief gr_mem
*
* For details refer to \ref gr_mem
*
*
* @{
*/


#ifndef IFX_BASE_MEM_H
#define IFX_BASE_MEM_H

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

/** @addtogroup gr_cat_SDK_base
  * @{
  */

/** @defgroup gr_mem Memory
  * @brief API for memory management
  *
  * Supports memory allocation and deallocation
  * as well as aligned allocation and aligned deallocation.
  *
  * @{
  */

/**
 * @brief Allocates memory of defined size.
 *
 * @param [in]     size      Number of bytes to be allocated.
 *
 * @return Pointer to the allocated memory if successful
 *         otherwise it returns NULL.
 */
IFX_DLL_PUBLIC
void* ifx_mem_alloc(size_t size);

/**
 * @brief Allocates memory for an array of defined number of elements
 *        with specified element size and initializes each byte to zero.
 *
 * @param [in]     count               Number of elements to be allocated.
 * @param [in]     element_size        Size of one element of the array.
 *
 * @return Pointer to the allocated memory if successful
 *         otherwise it returns NULL.
 */
IFX_DLL_PUBLIC
void* ifx_mem_calloc(size_t count,
                     size_t element_size);

/**
 * @brief Allocates memory of defined size with specified alignment.
 *        The size must be a multiple of alignment.
 *
 * @param [in]     size                Number of bytes to be allocated.
 * @param [in]     alignment           The number the allocated memory is aligned to.
 *
 * @return Pointer to the allocated memory if successful
 *         otherwise it returns NULL.
 */
IFX_DLL_PUBLIC
void* ifx_mem_aligned_alloc(size_t size,
                            size_t alignment);

/**
 * @brief Deallocates the memory which has been allocated by \ref ifx_mem_alloc
 *        or \ref ifx_mem_calloc. Do not use this to deallocate memory allocated
 *        with \ref ifx_mem_aligned_alloc.
 *
 * @param [in]     mem       Pointer to the memory to be deallocated.
 */
IFX_DLL_PUBLIC
void ifx_mem_free(void* mem);

/**
 * @brief Deallocates the memory which has been allocated by \ref ifx_mem_aligned_alloc.
 *        Do not use this to deallocate memory allocated allocated with \ref ifx_mem_alloc
 *        or \ref ifx_mem_calloc.
 *
 * @param [in]     mem       Pointer to the memory to be deallocated.
 */
IFX_DLL_PUBLIC
void ifx_mem_aligned_free(void* mem);

/**
 * @brief  Returns the number of the total allocated memory in bytes.
 *         Internal tracking must be enabled with TRACK_MEMORY_ALLOCATION
 *         defined to enable this function.
 *
 * @return Total number of bytes allocated. If tracking is not enabled
 *         this function returns -1.
 */
IFX_DLL_PUBLIC
int ifx_mem_get_total_alloc_size(void);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_MEM_H */

/**
* @}
*/