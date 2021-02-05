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
 * @file app_raw_data.c
 *
 * @brief Raw data application main program source file.
 * 
 */

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "ifxRadar/SDK.h"
#include "raw_data_defaults.h"

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/

#define NUM_FETCHED_FRAMES  10  /**< Number of frames to fetch */

/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
*/

/*
==============================================================================
   4. DATA
==============================================================================
*/

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/

void process_frame(ifx_Frame_R_t* frame);

void process_antenna_data(ifx_Matrix_R_t* antenna_data);

/*
==============================================================================
   6. LOCAL FUNCTIONS
==============================================================================
*/

/**
 * This function will separate different antenna signals
 * and pass them for further processing.
 * @param frame The frame may contain multiple antenna signals,
 *              depending on the device configuration.
 *              Each antenna signal can contain multiple chirps.
 */
void process_frame(ifx_Frame_R_t* frame)
{
    for(int i=0; i < frame->num_rx; i++)
    {
        process_antenna_data(frame->rx_data[i]);
    }
}

//----------------------------------------------------------------------------

/**
 * This function is an example showing a possible way
 * of processing antenna signal. The goal in this example is to
 * sum up all chirps into one vector.
 * @param antenna_data data from one antenna containing multiple chirps
 */
void process_antenna_data(ifx_Matrix_R_t* antenna_data)
{
    ifx_Vector_R_t chirp = {0};
    // Create the sum vector
    ifx_Vector_R_t* sum = ifx_vec_create_r(antenna_data->cols);

    // Iterate through all chirps
    for(uint32_t i=0; i < IFX_MAT_ROWS(antenna_data); i++)
    {
        // Fetch a chirp from the antenna data matrix
        ifx_mat_get_rowview_r(antenna_data, i, &chirp);
        // add it to the sum vector
        ifx_vec_add_r(&chirp, sum, sum);
    }

    // Divide the sum vector element wise by number of chirps in the antenna data
    ifx_vec_scale_r(sum, 1.0f / IFX_MAT_ROWS(antenna_data), sum);

    for(uint32_t i=0; i < IFX_VEC_LEN(sum); i++)
    {
        printf("%.4f ", IFX_VEC_AT(sum, i));
    }
    printf("\n\n");
    ifx_vec_destroy_r(sum);
}

/*
==============================================================================
   7. MAIN METHOD
==============================================================================
 */

int main(int argc, char** argv)
{
    ifx_Error_t error;
    ifx_Device_Config_t device_config = {0};
    ifx_Device_Handle_t device_handle = NULL;
    int frame_number = 0;
    printf("Radar SDK Version: %s\n", ifx_radar_sdk_get_version_string());

    /* Setup the device configuration structure containing all radar parameters, 
       to be used later to configure the device. */
    device_config.num_samples_per_chirp   = IFX_NUM_SAMPLES_PER_CHIRP;
    device_config.num_chirps_per_frame    = IFX_NUM_CHIRPS_PER_FRAME;
    device_config.sample_rate_Hz          = IFX_SAMPLE_RATE_HZ;
    device_config.frame_repetition_time_s = IFX_FRAME_REPETITION_TIME_S;
    device_config.lower_frequency_Hz      = IFX_LOWER_FREQUENCY_HZ;
    device_config.upper_frequency_Hz      = IFX_UPPER_FREQUENCY_HZ;
    device_config.tx_power_level          = IFX_TX_POWER_LEVEL;
    device_config.rx_mask                 = IFX_RX_MASK;
    device_config.tx_mask                 = IFX_TX_MASK;
    device_config.mimo_mode               = IFX_MIMO_OFF;
    device_config.chirp_repetition_time_s = IFX_CHIRP_REPETITION_TIME_S;
    device_config.if_gain_dB              = IFX_IF_GAIN_DB;

    /* Open the device */
    device_handle = ifx_device_create();
    if ((error = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to open device: %s\n", ifx_error_to_string(error));
        return EXIT_FAILURE;
    }

    /* Apply the device settings based on the device configuration structure. */
    ifx_device_set_config(device_handle, &device_config);
    if ((error = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to set device config:  %s\n", ifx_error_to_string(error));
        ifx_device_destroy(device_handle);
        return EXIT_FAILURE;
    }

    /* Create a frame structure for time domain data acquisition using information from the device handle. */
    ifx_Frame_R_t* frame = ifx_device_create_frame_from_device_handle(device_handle);
    if(frame == NULL)
    {
        fprintf(stderr, "Failed to create frame from device handle: %s\n", ifx_error_to_string(ifx_error_get()));
        return EXIT_FAILURE;
    }

    /* Loop over the number of frames to fetch, defined by NUM_FETCHED_FRAMES.
       The number of frames can be altered as desired for the intended target application. */
    while(frame_number < NUM_FETCHED_FRAMES)
    {
        ifx_Error_t ret = ifx_device_get_next_frame(device_handle, frame);
        if(ret != IFX_OK)
            break;

        frame_number++;

        process_frame(frame);
    }

    /* Close the device after processing all frames. */
    if(device_handle) {
        ifx_device_destroy(device_handle);
        device_handle = NULL;
        ifx_frame_destroy_r(frame);
    }

    return EXIT_SUCCESS;
}
