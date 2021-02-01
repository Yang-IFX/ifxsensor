#pragma once
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
 * @file app_common.h
 *
 * @brief This file defines functionality common for rdk apps.
 *
 */

#ifndef APP_COMMON_FUNCS_H
#define APP_COMMON_FUNCS_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

    /*
    ==============================================================================
       1. INCLUDE FILES
    ==============================================================================
    */

#include "json.h"

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

    typedef struct {
        const char* app_description; /**< Brief description of app shown in usage */
        const char* app_epilog;      /**< Additional text at the end of usage */

        ifx_Device_Metrics_t* default_metrics; /**< Default metrics used if no device config is given */

        ifx_Error_t (*app_init)(void * app_context);
        ifx_Error_t (*app_config)(void * app_context, ifx_json_t* json, ifx_Device_Config_t* device_config);
        ifx_Error_t (*app_process)(void* segmentation_context, ifx_Frame_R_t* frame);
        ifx_Error_t (*app_cleanup)(void* app_context);
    } app_t;


    /*
    ==============================================================================
       4. FUNCTION PROTOTYPES
    ==============================================================================
    */

    /**
     * @brief This is the common framework intended to be used for all apps. includes
     * argument parsing and device configuration calls to app_specific fucntions
     *
     * @param [in]     argc                 argument count transferred from the main function
     *
     * @param [in]     argv                 argument conditions transferred from the main function
     *
     * @param [in]     app_skeleton         structure containing init, config, run, clear function pointers
     *
     * @param [in]     app_context          structure containing app specific parameters
     *
     * @return Success/Error
     */

    int app_start(int argc, char** argv, app_t* app_skeleton, void* app_context);

    /**
     * @brief This function uses the Windows/Unix Audio library to play the file stored in wav format in the system
     *
     * @param [in]     filename             path and name of the file to be played.
     *
     * @return Success/Error
     */
    bool app_playaudio(const char*);

    /**
     * @brief This function can be called by the apps to print detailed run/debug information which will be active only
     *        in verbose mode triggered by argument '-v'
     *
     * @param [in]     message             message and text formatting.
     *
     * @param [in]     ...                 parameters used in formatted text
     *
     * @return none
     */
    void app_verbose(const char* message, ...);

    /**
    * @brief This function can be called by the apps to print the app running timestamp in hours:minutes:seconds format.
    *        The timer starts on app_start.
    *
    * @param [in]     none
    *
    * @return none
    */
    void app_printtime(void);

    /**
    * @brief This function can be called by the apps to print run/debug information which will be active only
    *        also in non verbose mode
    *
    * @param [in]     message             message and text formatting.
    *
    * @param [in]     ...                 parameters used in formatted text
    *
    * @return none
    */
    void app_print(const char* fmt, ...);

    /**
    * @brief This function can be called by the apps to print the timestamp in milliseconds. The time is not the
    *        actual time during record playback, but is calculated from the frame rate.
    *
    * @param [in]     none
    *
    * @return none
    */
    void app_print_timestamp(void);


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // #ifndef APP_COMMON_FUNCS_H 
