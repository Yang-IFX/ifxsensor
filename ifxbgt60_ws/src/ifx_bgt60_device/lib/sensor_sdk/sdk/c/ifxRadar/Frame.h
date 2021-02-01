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
 * @file Frame.h
 *
 * \brief \copybrief gr_frame
 *
 * For details refer to \ref gr_frame
 */

#ifndef IFX_RADAR_FRAME_H
#define IFX_RADAR_FRAME_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdbool.h>

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
 * @brief Defines the structure for a complete real frame of time domain data.
 *
 * When time domain data is acquired by a radar sensor device, it is copied into an instance of
 * this structure. The structure contains one matrix for each virtual active Rx antenna.
 */
typedef struct
{
    bool             complete; /**< Flag that indicates if the frame is complete. */
    uint8_t          num_rx;   /**< The number of Rx matrices in this instance (corresponds to the number
                                    of enabled Rx antennas multiplied by the number of enabled Tx antennas
                                    in the radar device).*/
    ifx_Matrix_R_t** rx_data;  /**< This is an array of real data matrices. It contains num_rx elements.*/
} ifx_Frame_R_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Device_Access
  * @{
  */

/** @defgroup gr_frame Frame
  * @brief API for Measurement Frame handling
  *
  * Supports operations such as creation and destruction of frames.
  *
  * @{
  */

/**
 * @brief Creates a frame structure for time domain real data acquisition
 *
 * This function initializes a data structure (of type \ref ifx_Frame_R_t) that can hold a time domain
 * data frame according to the dimensions provided as parameters. These parameters can be obtained for
 * instance by parsing through a pre-recorded file. If a device is connected then the
 * \ref ifx_device_create_frame_from_device_handle should be used instead of this function, as that function
 * reads the dimensions from configured the device handle.
 *
 * @param [in]     num_antennas                  Number of Rx antennas available in the device
 * @param [in]     num_chirps_per_frame          Number of chirps configured in a frame
 * @param [in]     num_samples_per_chirp         Number of samples per chirp
 *
 * @return Pointer to allocated and initialized frame structure \ref ifx_Frame_R_t or NULL if allocation failed.
 */
IFX_DLL_PUBLIC
ifx_Frame_R_t* ifx_frame_create_r(uint8_t num_antennas,
                                  uint32_t num_chirps_per_frame,
                                  uint32_t num_samples_per_chirp);

/**
 * @brief Frees the memory allocated for the real frame structure.
 *
 * This function frees the memory associated with the provided frame structure and un-initializes
 * it. This function must be called when the frame is not used any longer to avoid memory leaks. *frame*
 * must not be used any more after this function was called.
 *
 * This function frees the memory for the frame handle and the internal array of matrices.
 *
 * @param [in]     frame     The real frame structure to be deallocated.
 */
IFX_DLL_PUBLIC
void ifx_frame_destroy_r(ifx_Frame_R_t* frame);

/**
 * @brief Check if the frame is complete.
 *
 * This function returns true if the given frame is complete, otherwise false.
 * The function is useful in combination with \ref ifx_device_get_next_frame_nonblock
 * to check if all samples of the current frame have been received.
 *
 * @param [in]     frame     Pointer to time domain data real frame structure.
 * @retval         true      if the frame is complete.
 * @retval         false     if the frame is not complete.
 */
IFX_DLL_PUBLIC
bool ifx_frame_complete(const ifx_Frame_R_t* frame);
/**
 * @brief Gets real matrix from antenna.
 *
 * @param [in]     frame     Pointer to time domain data real frame structure.
 * @param [in]     antenna   Rx antenna.
 * @return Pointer to real matrix containing time domain data corresponding to the specified Rx antenna.
 */
IFX_DLL_PUBLIC
ifx_Matrix_R_t* ifx_frame_get_mat_from_antenna_r(ifx_Frame_R_t* frame, 
                                                 uint8_t antenna);

/**
  * @}
  */

/**
  * @}
  */
 
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_FRAME_H */
