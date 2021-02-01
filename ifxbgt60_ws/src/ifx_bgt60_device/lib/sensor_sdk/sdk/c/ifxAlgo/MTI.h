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
 * @file MTI.h
 *
 * \brief \copybrief gr_mti
 *
 * For details refer to \ref gr_mti
 */

#ifndef IFX_ALGO_MTI_H
#define IFX_ALGO_MTI_H

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
 * @brief A handle for an instance of MTI module, see MTI.h.
 */
typedef struct ifx_MTI_s* ifx_MTI_Handle_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_mti Moving Target Indicator (MTI)
  * @brief API for Moving Target Indicator (MTI) filter.
  * 
  * An algorithm explanation is also available at the \ref ssct_radarsdk_algorithms_mti SDK documentation.
  * 
  * @{
  */

/**
 * @brief Creates a new MTI filter
 *
 * Instantiate an MTI filter \ref ifx_MTI_Handle_t which contains information 
 * required to perform MTI (Moving Target Indication) filtering. 
 *
 * @param [in]     alpha_mti_filter    Filter coefficient. Valid between 0.0 and 1.0
 * @param [in]     spectrum_length     Length of input / output vectors used within MTI filter
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_MTI_Handle_t ifx_mti_create(const ifx_Float_t alpha_mti_filter,
                                const uint32_t spectrum_length);

/**
 * @brief Destroys the MTI filter handle.
 *
 * @param [in,out] handle    A handle to the MTI filter object
 *
 */
IFX_DLL_PUBLIC
void ifx_mti_destroy(ifx_MTI_Handle_t handle);

/**
 * @brief Uses a valid MTI filter handle to filter an input vector of real values.
 *
 * @param [in]     handle    A handle to the MTI filter object
 * @param [in]     input     Real value vector used as an input for MTI filter
 * @param [out]    output    Real value vector used as an output of MTI filter
 *
 */
IFX_DLL_PUBLIC
void ifx_mti_run(ifx_MTI_Handle_t handle,
                 const ifx_Vector_R_t* input,
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

#endif /* IFX_ALGO_MTI_H */
