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
* \file Window.h
*
* \brief   \copybrief gr_window
*
* For details refer to \ref gr_window
*
*
* @{
*/


#ifndef IFX_ALGO_WINDOW_H
#define IFX_ALGO_WINDOW_H

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
 * @brief Defines supported Window options.
 */
typedef enum
{
    IFX_WINDOW_HAMM           = 0U,    /**< Hamming window */
    IFX_WINDOW_HANN           = 1U,    /**< Hanning window */
    IFX_WINDOW_BLACKMANHARRIS = 2U,    /**< Blackmann Harris window */
    IFX_WINDOW_CHEBYSHEV      = 3U,    /**< Chebyshev window */
    IFX_WINDOW_BLACKMAN       = 4U,    /**< Blackman window */
} ifx_Window_Type_t;

/**
 * @brief Defines the structure for Window module related settings.
 */
typedef struct
{
    ifx_Window_Type_t type;  /**< Type of window function defined by \ref ifx_Window_Type_t */
    uint32_t          size;  /**< Number of elements in the window */
    ifx_Float_t       at_dB; /**< Attenuation parameter, in case of Chebyshev window.
                                  Defines the attenuation in dBs required to generate
                                  the pass band ripple for a Chebyshev window.
                                  This must be a positive number. */
    ifx_Float_t       scale; /**< Scale factor of all elements inside the window */
} ifx_Window_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Signal_Processing
  * @{
  */
 
/** @defgroup gr_window Window
  * @brief API for Window function processing
  *
  * Supports various Window functions and their coefficients:
  * - Hamming
  * - Hanning
  * - Blackman
  * - Blackman-Harris
  * - Chebyshev
  *
  * Note: Dolph Chebyshev Window is implemented according to:
  * http://practicalcryptography.com/miscellaneous/machine-learning/implementing-dolph-chebyshev-window/
  *
  * @{
  */

/**
 * @brief Generates the coefficients of a user selected window for a given length.
 *
 * @param [in]     config    \ref ifx_Window_Config_t "Window configuration structure" defining the type of window,
 *                           the number of elements in the window and also
 *                           any additional parameter specific to certain window type.
 * @param [in,out] win       Pointer to an allocated and populated vector instance defined by \ref ifx_Vector_R_t
 *                           filled with Window coefficients defined in \ref ifx_Window_Config_t.
 */
IFX_DLL_PUBLIC
void ifx_window_init(const ifx_Window_Config_t* config,
                     ifx_Vector_R_t* win);

/**
  * @}
  */
 
/**
  * @}
  */ 
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_WINDOW_H */

/**
* @}
*/