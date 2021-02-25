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
* \file AngleMonopulse.h
*
* \brief   \copybrief gr_anglemonopulse
*
* For details refer to \ref gr_anglemonopulse
*
*
* @{
*/


#ifndef IFX_RADAR_ANGLE_MONOPULSE_H
#define IFX_RADAR_ANGLE_MONOPULSE_H

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
 * @brief A handle for an instance of AngleMonopulse module, see AngleMonopulse.h
 */
typedef struct ifx_AngleMonopulse_s* ifx_AngleMonopulse_Handle_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_anglemonopulse Angle Monopulse
  * @brief API for Phase Monopulse algorithm
  *
  * Phase Monopulse algorithm is used for angle calculation.
  *
  * Interface provides as input:
  * - A vector of desired range/doppler FFT cells (complex number) for antenna x
  * - A vector of desired range/doppler FFT cells (complex number) for antenna y
  * - Each entry of the two vectors represents the same detected target we want to have the angle information for
  * Interface provides following configuration parameter:
  * - wavelength or center frequency
  * - distance of the two antennas
  * Interface provides as output:
  * - vector of angles in degrees for each row (target) the two input vectors
  *
  * @{
  */

/**
 * @brief Creates an angle monopulse handle, to contain two key parameters i.e. wavelength and antenna spacing.
 *
 * @param [in]     wavelength          Wavelength (in units of meters), computed from center frequency and speed of light
 * @param [in]     antenna_spacing     Physical spacing between antennas on BGT radar chip (in units of meters)
 *
 * @return Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_AngleMonopulse_Handle_t ifx_anglemonopulse_create(const ifx_Float_t wavelength,
                                                      const ifx_Float_t antenna_spacing);

/**
 * @brief Destroys the angle monopulse handle.
 *
 * @param [in,out] handle    A handle to the angle monopulse object
 *
 */
IFX_DLL_PUBLIC
void ifx_anglemonopulse_destroy(ifx_AngleMonopulse_Handle_t handle);

/**
 * @brief Based on complex input scalar from two receiver antennas, a corresponding
 *        angle scalar (in units of degrees) is calculated using phase monopulse algorithm.
 *
 * @param [in]     handle    A handle to the angle monopulse object
 * @param [in]     rx1       First Rx antenna used as numerator argument in algorithm
 * @param [in]     rx2       Second Rx antenna used as denominator argument in algorithm
 *
 * @return Real value corresponding to the angle in degrees for a given inputs Rx1 & Rx2.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_anglemonopulse_scalar_run(const ifx_AngleMonopulse_Handle_t handle,
                                          ifx_Complex_t rx1,
                                          ifx_Complex_t rx2);

/**
 * @brief Based on complex input vector from two receiver antennas, a corresponding
 *        angle vector (in units of degrees) is calculated using phase monopulse algorithm.
 *
 * @param [in]     handle              A handle to the angle monopulse object
 * @param [in]     rx1                 First Rx antenna used as numerator argument in algorithm
 * @param [in]     rx2                 Second Rx antenna used as denominator argument in algorithm
 * @param [out]    target_angle_deg    Output vector of real values corresponding to the angle (in units of degrees)
 *                                     for a given input Rx1 & Rx2 vector
 *
 */
IFX_DLL_PUBLIC
void ifx_anglemonopulse_vector_run(const ifx_AngleMonopulse_Handle_t handle,
                                   const ifx_Vector_C_t* rx1,
                                   const ifx_Vector_C_t* rx2,
                                   ifx_Vector_R_t* target_angle_deg);

/**
 * @brief Sets the new value of wavelength used in angle calculation
 *
 * @param [in]     handle              A handle to the angle monopulse object
 * @param [in]     wavelength          New value of wavelength (in units of meters) to be used by the monopulse algorithm
 *
 */
IFX_DLL_PUBLIC
void ifx_anglemonopulse_set_wavelength(ifx_AngleMonopulse_Handle_t handle,
                                       const ifx_Float_t wavelength);

/**
 * @brief Returns the current value of wavelength used in angle calculation
 *
 * @param [in]     handle              A handle to the angle monopulse object
 *
 * @return Value of wavelength (in units of meters) used by the algorithm.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_anglemonopulse_get_wavelength(const ifx_AngleMonopulse_Handle_t handle);

/**
 * @brief Sets the new value of antenna spacing used in angle calculation
 *
 * @param [in]     handle              A handle to the angle monopulse object
 * @param [in]     antenna_spacing     New value of antenna spacing (in units of meters) to be used by the monopulse algorithm
 *
 */
IFX_DLL_PUBLIC
void ifx_anglemonopulse_set_antenna_spacing(ifx_AngleMonopulse_Handle_t handle,
                                            const ifx_Float_t antenna_spacing);

/**
 * @brief Returns the current value of antenna spacing used in angle calculation
 *
 * @param [in]     handle    A handle to the angle monopulse object
 *
 * @return Value of antenna spacing (in units of meters) used by the algorithm.
 *
 */
IFX_DLL_PUBLIC
ifx_Float_t ifx_anglemonopulse_get_antenna_spacing(const ifx_AngleMonopulse_Handle_t handle);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_ANGLE_MONOPULSE_H */

/**
* @}
*/