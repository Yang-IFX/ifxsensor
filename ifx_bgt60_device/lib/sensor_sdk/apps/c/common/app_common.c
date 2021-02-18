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
* \file app_common.c
*
* \brief   This file contains repeating functionality common for rdk apps.
*
*
* @{
*/


// disable warnings about unsafe functions with MSVC
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#if defined(_WIN32) || defined(__MINGW64__) || defined(__CYGWIN__)
    /* use Windows audio system */
#define AUDIO_WIN32
#endif

 /*
 ==============================================================================
    1. INCLUDE FILES
 ==============================================================================
 */

#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>

#include "ifxRadar/SDK.h"
#include "app_common.h"
#include "util.h"
#include "argparse.h"
#include "time_formatter.h"
#include "json.h"

#ifdef AUDIO_WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

 /*
 ==============================================================================
    2. LOCAL DEFINITIONS
 ==============================================================================
 */
//  SUPPORTED RECORDING FORMATS
//---------------------------------
//  RECORD_FORMAT_DEFAULT [one sample per line, one empty line after each chirp]
//==============================
//  Ant0_Chirp_0_samples
//
//  Ant0_Chirp_1_samples
//
//  ....
//  Ant0_Chirp_last_samples
//
//  Ant1_Chirp_0_samples
//
//  Ant1_Chirp_1_samples
//  .... till last antenna Data for each frame

//  RECORD_FORMAT_ANTENNA_TABLE [sample index and corresponding sample for each antennae in every line]
//==============================
//  0, Ant_0_Chirp_0_sample_0,Ant_1_Chirp_0_sample_0, ... ,Ant_last_Chirp_0_sample_0,
//  1, Ant_0_Chirp_0_sample_1,Ant_1_Chirp_0_sample_1, ... ,Ant_last_Chirp_0_sample_1,
//  ....
//  last,Ant_0_Chirp_0_sample_last,Ant_1_Chirp_0_sample_last, ... ,Ant_last_Chirp_0_sample_last,
//  0,Ant_0_Chirp_1_sample_0, Ant_1_Chirp_1_sample_0, ... ,Ant_last_Chirp_1_sample_0,
//  1,Ant_0_Chirp_1_sample_1, Ant_1_Chirp_1_sample_1, ... ,Ant_last_Chirp_1_sample_1,
//  ....
//  last,Ant_0_Chirp_1_sample_last, Ant_1_Chirp_1_sample_last, ... ,Ant_last_Chirp_1_sample_last,
//  ...
//  ...
//  0,Ant_0_Chirp_last_sample_0,Ant_1_Chirp_last_sample_0, ... ,Ant_last_Chirp_last_sample_0,
//  1,Ant_0_Chirp_last_sample_1,Ant_1_Chirp_last_sample_1, ... ,Ant_last_Chirp_last_sample_1,
//  ....
//  last,Ant_0_Chirp_last_sample_last,Ant_1_Chirp_last_sample_last, ... ,Ant_last_Chirp_last_sample_last,

#define RECORD_FORMAT_DEFAULT                   0
#define RECORD_FORMAT_ANTENNA_TABLE             1

/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
*/



/*
==============================================================================
   4. LOCAL DATA
==============================================================================
*/
static  struct {
    bool verbose; // detailed information printout
    bool sound; // Enable sounds if applicable
    ifx_Time_Handle_t time_handle;
    FILE* file_results;
    volatile bool is_running;
} app_common;

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/


/*
==============================================================================
   6. LOCAL FUNCTIONS
==============================================================================
*/

/** @brief Finds a program in the search path
 *
 * Check the search path given by the environment variable $PATH for a file
 * named program. If the program could be found, the path to the program is
 * returned. Otherwise, NULL is returned.
 *
 * @note You have to free the memory for the returned string.
 *
 * @param [in]     program   Name of program
 * @retval pathname to program if program was found
 * @retval NULL if the program was not found in the search path
 */
#ifndef AUDIO_WIN32
char* find_program(const char* program)
{
    if (program == NULL || strlen(program) == 0)
        return NULL;

    const size_t len_program = strlen(program);
    const char* env_path = getenv("PATH");

    if (env_path==NULL)
        return NULL;

    while (1)
    {
        // if p is NULL we have the last item of the path
        char* p = strchr(env_path, ':');

        size_t len_directory;
        if (p == NULL)
            // length: until \0 byte
            len_directory = strlen(env_path);
        else
            // length: from p to the next colon
            len_directory = p - env_path;

        // required length: len_directory + / + len(program) + '\0'
        const size_t len_fullpath = len_directory + 1 + strlen(program) + 1;
        char* fullpath = malloc(len_fullpath * sizeof(char));
        if (fullpath == NULL)
            return NULL;

        memcpy(fullpath, env_path, len_directory);
        fullpath[len_directory] = '/';
        memcpy(fullpath + len_directory + 1, program, len_program);
        fullpath[len_directory + len_program + 1] = '\0';

        if (file_executable(fullpath))
            return fullpath;
        else
            free(fullpath);

        if (p == NULL)
            return NULL;

        env_path = p + 1;
    }
}
#endif /* AUDIO_WIN32 */

bool app_playaudio(const char* wavfile)
{
    if (app_common.sound) {
        if (wavfile == NULL || strlen(wavfile) == 0 || !file_readable(wavfile))
            return false;

#ifdef AUDIO_WIN32
        return PlaySound(wavfile, NULL, SND_ASYNC | SND_FILENAME);
#else
        static char* aplay;
        static bool init = false;

        // find path to aplay once
        if (!init)
        {
            aplay = find_program("aplay");
            init = true;
        }

        if (aplay == NULL)
            // aplay was not found
            return false;

        pid_t pid = fork();
        if (pid == -1)
            // problem while forking
            return false;
        else if (pid == 0)
        {
            // child

            // close stderr and stdout
            fclose(stdout);
            fclose(stderr);

            // exec aplay
            execl(aplay, aplay, wavfile, (char*)NULL);

            // if we are here an error occurred
            exit(1);
        }
        else
            // parent
            return true;
#endif /* AUDIO_WIN32 */
    }
    return false;
}

void app_verbose(const char* fmt, ...) {
    va_list args;

    va_start(args, fmt);
    if (app_common.verbose) {
        vfprintf(app_common.file_results,fmt, args);
    }
    va_end(args);
}

void app_print(const char* fmt, ...) {
    va_list args;

    va_start(args, fmt);
    vfprintf(app_common.file_results, fmt, args);
    va_end(args);
}

void app_printtime(void) {
    if (app_common.time_handle) {
        fprintf(app_common.file_results,"\"%s\"", ifx_time_get_cstr(app_common.time_handle));
    }
}


void signal_handler(int sig)
{
    if (sig == SIGINT)
        app_common.is_running = false;
}

static void printf_frame_to_file_r(FILE* f,
    ifx_Frame_R_t* frame)
{
    if (!frame) return;

    for (uint32_t i = 0; i < IFX_MAT_ROWS(frame->rx_data[0]); i++)
    {
        for (uint32_t j = 0; j < IFX_MAT_COLS(frame->rx_data[0]); j++)
        {
            fprintf(f, "%4d,", j);
            for (uint32_t k = 0; k < frame->num_rx; k++)
            {
                ifx_Float_t value = ifx_mat_get_element_r(ifx_frame_get_mat_from_antenna_r(frame, k), i, j);
                fprintf(f, "%.6f,", value);
            }
            fprintf(f, "\n");
        }
    }
}

void error_callback(const char* filename, const char* functionname, int line, ifx_Error_t error)
{
    fprintf(stderr, "File:     | %s\n", filename);
    fprintf(stderr, "Function: | %s\n", functionname);
    fprintf(stderr, "Line:     | %d\n", line);
    fprintf(stderr, "Reason:   | %s\n", ifx_error_to_string(error));
    fprintf(stderr, "Errorcode:| 0x%x\n", error);
}

int app_start(int argc, char** argv, app_t* application, void *app_context)
{
    char* record_file_path = NULL;
    char* data_file_path = NULL;
    char* config_file_path = NULL;
    char* result_file_path = NULL;
    char* device_port_name = NULL;
    char* device_uuid = NULL;
    char* app_name = NULL;

    bool buffer = false;
    struct argparse argparse;

    uint32_t time_limit = 0;
    uint32_t frame_limit = 0;
    int record_format = RECORD_FORMAT_DEFAULT;

    uint32_t frame_count;

    // initialize
    FILE* file_record = NULL;
    FILE* file_data = NULL;
    ifx_json_t* json = NULL;

    ifx_Device_Handle_t device_handle = NULL;
    ifx_Frame_R_t* frame = NULL;
    char *app_usage = NULL;
    char* self = argv[0];

    // this is the value we return to main if something goes wrong
    int exitcode = EXIT_FAILURE;

    //---------------- Usage Description String derivation -----------------------
    app_name = extract_filename_from_path(self);
    if (app_name == NULL)
    {
        fprintf(stderr, "Could not extract app name for description string \n");
        goto cleanup;
    }

    app_usage = calloc((strlen(app_name) + 12), sizeof(char));
    if (app_usage == NULL)
    {
        fprintf(stderr, "Could not allocate memory for app description string \n");
        goto cleanup;
    }

    sprintf(app_usage, "%s [OPTIONS]", app_name);
    const char *const usage_str[] = { app_usage, NULL };   // to be used in argparse


    // -------------------------------------------------------------------------------
    // -------------------------  Initialization  ------------------------------------
    // -------------------------------------------------------------------------------
    app_common.file_results = stdout; // by default print outputs to console

    app_common.time_handle = NULL;
    app_common.is_running = true;

    // parse commandline
    struct argparse_option options[] = {
        OPT_HELP(),
        OPT_GROUP("Options"),
        OPT_STRING('d', "data", &data_file_path, "data filename: recorded data to be used for app", NULL, 0, 0),
        OPT_STRING('c', "config", &config_file_path, "configuration filename: radar configuration to be used", NULL, 0, 0),
        OPT_STRING('r', "record", &record_file_path, "recording filename: records data to this file", NULL, 0, 0),
        OPT_INTEGER('R', "format", &record_format, "recording format default:0, antenna_table:1", NULL, 0, 0),
        OPT_STRING('o', "output", &result_file_path, "results filename: switches results display from stdout to file", NULL, 0, 0),
        OPT_STRING('p', "port", &device_port_name, "device port: attempt to connect to device on specified port", NULL, 0, 0),
        OPT_STRING('u', "uuid", &device_uuid, "device uuid: attempt to connect to device using specified uuid", NULL, 0, 0),
        OPT_BOOLEAN('b', "buffer", &buffer, "buffer output to stdout and stderr", NULL, 0, 0),
        OPT_BOOLEAN('s', "sound", &app_common.sound, "play sounds if applicable", NULL, 0, 0),
        OPT_BOOLEAN('v', "verbose", &app_common.verbose, "print detailed app output information", NULL, 0, 0),
        OPT_INTEGER('t', "time", &time_limit, "time in seconds to run", NULL, 0, 0),
        OPT_INTEGER('f', "frames", &frame_limit, "number of frames to run", NULL, 0, 0),
        OPT_END(),
    };

    argparse_init(&argparse, options, usage_str, 0);
    argparse_describe(&argparse, application->app_description, application->app_epilog);
    argparse_parse(&argparse, argc, argv);

    // disable buffering unless --buffer was given
    if (!buffer)
    {
        disable_buffering(stdout);
        disable_buffering(stderr);
    }

    if (device_port_name && device_uuid) {
        fprintf(stderr, "uuid and portname are mutually exclusive!\n");
        goto cleanup;
    }
    
    ifx_error_set_callback(error_callback);

    if (ifx_time_create(&app_common.time_handle))
    {
        fprintf(stderr, "Failed creating time handle!\n");
        goto cleanup;
    }

    // --------------------------------------------------------------------------
    // -------------------------  app specific init -----------------------------
    // --------------------------------------------------------------------------
    if(application->app_init(app_context) != IFX_OK)
        goto cleanup;

    //------------------------ Check File options ------------------------------
    if (record_file_path)
    {
        file_record = fopen(record_file_path, "w");
        if (file_record == NULL)
        {
            fprintf(stderr, "Could not open file %s for writing", record_file_path);
            goto cleanup;
        }
    }

    if (data_file_path)
    {
        file_data = fopen(data_file_path, "r");
        if (file_data == NULL)
        {
            fprintf(stderr, "Could not open file %s for reading", data_file_path);
            goto cleanup;
        }
    }

    if (result_file_path)
    {
        app_common.file_results = fopen(result_file_path, "w");
        if (app_common.file_results == NULL)
        {
            fprintf(stderr, "Could not open file %s for writing", result_file_path);
            goto cleanup;
        }
    }

    json = ifx_json_create();
    if (!json)
    {
        fprintf(stderr, "Cannot create JSON structure");
        goto cleanup;
    }

    ifx_Device_Config_t device_config = { 0 };
    if (config_file_path)
    {
        /* read configuration from json file */
        bool ret = ifx_json_load_from_file(json, config_file_path);
        if (!ret)
        {
            fprintf(stderr, "Error parsing configuration file %s: %s", config_file_path, ifx_json_get_error(json));
            goto cleanup;
        }

        if (ifx_json_has_config_single_shape(json))
        {
            ret = ifx_json_get_device_config_single_shape(json, &device_config);
            if (!ret)
            {
                fprintf(stderr, "Error parsing fmcw_single_shape configuration: %s", ifx_json_get_error(json));
                goto cleanup;
            }
        }
        else if (ifx_json_has_config_scene(json))
        {
            ifx_Device_Metrics_t scene_config;
            ret = ifx_json_get_device_config_scene(json, &scene_config);
            if (!ret)
            {
                fprintf(stderr, "Error parsing fmcw_scene configuration: %s", ifx_json_get_error(json));
                goto cleanup;
            }

            ifx_device_translate_metrics_to_config(NULL, &scene_config, &device_config);
            if (ifx_error_get() != IFX_OK)
            {
                fprintf(stderr, "Error converting scene to device configuration");
                goto cleanup;
            }
        }
    }
    else
    {
        /* use default configuration */
        ifx_device_translate_metrics_to_config(NULL, application->default_metrics, & device_config);

        /* and save it in the json structure*/
        ifx_json_set_device_config_single_shape(json, &device_config);
    }

    //missing parameters of device default configuration will be derived 
    //from given default metrics

    // --------------------------------------------------------------------------
    // ---------------------  app specific json config --------------------------
    // --------------------------------------------------------------------------

    if (application->app_config(app_context, json, &device_config) != IFX_OK)
        goto cleanup;

    // Write final configuration to file
    if (file_record)
    {
        // Prepare Config file write if record enabled
        const char* extension = "_config.json";
        char* record_config_file_path = calloc((strlen(record_file_path) + strlen(extension) + 1), sizeof(char));
        if (record_config_file_path == NULL)
        {
            fprintf(stderr, "Could not allocate memory for config filename\n");
            goto cleanup;
        }
        strcat(record_config_file_path, record_file_path);
        strtok(record_config_file_path, ".");
        strcat(record_config_file_path, extension);

        ifx_json_save_to_file(json, record_config_file_path);

        free(record_config_file_path);
    }

    // --------------------------------------------------------------------------
    // ---------------------  Initialize Device ---------------------------------
    // --------------------------------------------------------------------------

    if (data_file_path == NULL)
    {
        if (device_uuid) {
            uint8_t uuid[16] = { 0 };

            if (!get_uuid_from_string(device_uuid, uuid))
            {
                fprintf(stderr, "Invalid uuid\n");
                goto cleanup;
            }
            device_handle = ifx_device_create_by_uuid(uuid);
        }
        else
            device_handle = ifx_device_create_by_port(device_port_name);

        if (ifx_error_get() != IFX_OK)
        {
            fprintf(stderr, "Failed to open Device. (%x)\n", ifx_error_get());
            goto cleanup;
        }

        // set configuration
        ifx_device_set_config(device_handle, &device_config);

        if (ifx_error_get() != IFX_OK)
        {
            fprintf(stderr, "Failed to initialize Device. (%x)\n", ifx_error_get());
            goto cleanup;
        }
    }


    if (file_data == NULL)
    {
        frame = ifx_device_create_frame_from_device_handle(device_handle);
        if (ifx_error_get())
            goto cleanup;
    }
    else
    {
        // get antenna count from configuration
        uint8_t rx_antenna_count = ifx_device_count_rx_antennas(&device_config);
        frame = ifx_frame_create_r(rx_antenna_count,
            device_config.num_chirps_per_frame,
            device_config.num_samples_per_chirp);
    }

    // change to the directory of the executable
    // in order to access resources (e.g. sound files) which use relative paths
    change_to_dirname(self);

    printf("Radar SDK Version: %s\n", ifx_radar_sdk_get_version_string());

    // install signal handler
    signal(SIGINT, signal_handler);

    frame_count = 0;

    while (app_common.is_running)
    {
        ifx_Error_t ret;

        if (file_data)
        {
            bool is_eof = false;
            for (uint8_t i = 0; i < frame->num_rx; i++)
            {
                if ((is_eof = !get_matrix_from_file_r(file_data, frame->rx_data[i])))
                    break;
            }
            if (is_eof)
                break;

            // increase frame count
            frame_count++;
        }
        else {
            ret = ifx_device_get_next_frame(device_handle, frame);

            /* in case of a timeout we read to fast, so we should just try again */
            if (ret == IFX_ERROR_TIMEOUT)
                continue;

            /* FIFO overflow occured */
            if (ret == IFX_ERROR_FIFO_OVERFLOW)
            {
                fprintf(stderr, "FIFO overflow\n");
                // In case of overflow this frame can be ignored (continue)
                // but when recording is enabled it is necessary not to lose frames
                if (file_record)
                {
                    fprintf(stderr, "Recording not valid. Abort!\n");
                    fprintf(file_record, "\nFIFO Overflow. Abort!\n");
                    goto cleanup;
                }
                continue;
            }
            else if (ret != IFX_OK)
            {
                fprintf(stderr, "Error getting next frame: %s (%d)\n", ifx_error_to_string(ret), ret);
                goto cleanup;
            }

            // increase frame count
            frame_count++;
        }
        if (file_record) {
            if (record_format == RECORD_FORMAT_ANTENNA_TABLE) {
                printf_frame_to_file_r(file_record, frame);
            }
            else { // RECORD FORMAT_DEFAULT
                for (uint8_t i = 0; i < frame->num_rx; i++)
                {
                    print_matrix_to_file_r(file_record, frame->rx_data[i]);
                }
            }
        }
        if (ifx_error_get() != IFX_OK)
            goto cleanup;
        // --------------------------------------------------------------------------
        // ---------------------  app specific functionality ------------------------
        // --------------------------------------------------------------------------
        app_print("{ \"elapsed_time\":\"%s\", \"frame_number\":%d",
            ifx_time_get_cstr(app_common.time_handle),frame_count);
        if (application->app_process(app_context, frame) != IFX_OK) {
            app_print(" }\n");
            goto cleanup;
        }
        app_print(" }\n");

        // ---------------------  Exit conditions ----------------------------------
        if (frame_limit > 0 && frame_count == frame_limit)
        {
            app_common.is_running = false;
            printf("frame limit reached.\n");
        }

        if (time_limit > 0)
        {
            if (device_config.frame_repetition_time_s * frame_count >= time_limit)
            {
                app_common.is_running = false;
                printf("time limit reached.\n");
            }
        }
    }

    // everything successful
    exitcode = EXIT_SUCCESS;
cleanup:
    // --------------------------------------------------------------------------
    // --------------------------------  common ---------------------------------
    // --------------------------------------------------------------------------
    if (app_common.time_handle)
    {
        ifx_time_destroy(app_common.time_handle);
        app_common.time_handle = NULL;
    }

    if(frame)
        ifx_frame_destroy_r(frame);

    if (device_handle)
    {
        fprintf(stderr, "Closing Device\n");
        ifx_device_destroy(device_handle);
        device_handle = NULL;
    }
    fflush(stdout);

    ifx_json_destroy(json);
    free(app_usage);

    if (file_data)
        fclose(file_data);
    if (file_record)
        fclose(file_record);
    if (app_common.file_results)
        fclose(app_common.file_results);

    // --------------------------------------------------------------------------
    // --------------------------------  app specific ---------------------------
    // --------------------------------------------------------------------------
    application->app_cleanup(app_context);


    return exitcode;
}


//----------------------------------------------------------------------------


/*
==============================================================================
   7. EXPORTED FUNCTIONS
==============================================================================
*/


//----------------------------------------------------------------------------

/**
* @}
*/