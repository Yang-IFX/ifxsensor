#pragma once
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
*
* @{
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

/**
* @}
*/
