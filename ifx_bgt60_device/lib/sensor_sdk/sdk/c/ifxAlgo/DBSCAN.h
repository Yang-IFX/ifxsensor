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
* \file DBSCAN.h
*
* \brief   \copybrief gr_dbscan
*
* For details refer to \ref gr_dbscan
*
*
* @{
*/


#ifndef IFX_ALGO_DBSCAN_H
#define IFX_ALGO_DBSCAN_H

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
 * @brief A handle for an instance of DBSCAN module, see DBSCAN.h
 */
typedef struct ifx_DBSCAN_s* ifx_DBSCAN_Handle_t;

/**
 * @brief Defines the structure for DBSCAN module related settings.
 */
typedef struct
{
   uint8_t     min_points;         /**< Minimum number of neighbor points to be recognized as a cluster.*/
   ifx_Float_t min_dist;           /**< Minimum distance at which a point is recognized as a neighbor.*/
   uint32_t    max_num_detections; /**< Maximum number of detections (points) which can appear.*/
} ifx_DBSCAN_Config_t;


/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Algorithms
  * @{
  */

/** @defgroup gr_dbscan DBSCAN
  * @brief API for Density-based spatial clustering of applications with noise (DBSCAN) algorithm
  *
  * This algorithm is used to perform Density-based spatial clustering
  * of applications with noise (DBSCAN).
  *
  * An algorithm explanation is available at the \ref ssct_radarsdk_algorithms_detect_dbscan SDK documentation.
  *
  * @{
  */

/**
 * @brief Creates an instance of the DBSCAN handle with given configuration.
 *
 * @param [in]     config    Configuration options of the DBSCAN object.
 * 
 * @return  Handle to the newly created instance or NULL in case of failure.
 *
 */
IFX_DLL_PUBLIC
ifx_DBSCAN_Handle_t ifx_dbscan_create(ifx_DBSCAN_Config_t* config);

/**
 * @brief Releases all resources held by the given handle.
 * 
 * @param [in,out] handle    A handle to the DBSCAN object.
 *
 */
IFX_DLL_PUBLIC
void ifx_dbscan_destroy(ifx_DBSCAN_Handle_t handle);

/**
 * @brief Performs the DBSCAN (Density-based spatial clustering of applications with noise)
 * algorithm on given detections.
 *
 * @param [in]     handle              A handle to the DBSCAN object.
 * @param [in]     detections          The detection points to search for clusters in. Note that detections
 *                                     must store the coordinates interleaved (x1, y1, x2, y2,..., xn, yn).
 *                                     Therefore this parameter must point to valid memory of the size
 *                                     2 * num_detections * sizeof(type).
 * @param [in]     num_detections      Number of detection points.
 * @param [out]    cluster_vector      The output vector will consist the vector number for each detection.
 *                                     This vector must point to valid memory of minimum num_detection elements.
 */
IFX_DLL_PUBLIC
void ifx_dbscan_run(ifx_DBSCAN_Handle_t handle,
                    uint16_t* detections,
                    uint16_t num_detections,
                    uint16_t* cluster_vector);

/**
 * @brief Sets the min points attribute see \ref ifx_DBSCAN_Config_t.
 *
 * @param [in]     handle              A handle to the DBSCAN object.
 * @param [in]     min_points          The minimum points to set.
 *
 */
IFX_DLL_PUBLIC
void ifx_dbscan_set_min_points(ifx_DBSCAN_Handle_t handle,
                               uint16_t min_points);

/**
 * @brief Sets the min distance attribute see \ref ifx_DBSCAN_Config_t.
 *
 * @param [in]     handle              A handle to the DBSCAN object.
 * @param [in]     min_distance        The minimum distance to set.
 *
 */
IFX_DLL_PUBLIC
void ifx_dbscan_set_min_distance(ifx_DBSCAN_Handle_t handle,
                                 ifx_Float_t min_distance);

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_ALGO_DBSCAN_H */

/**
* @}
*/