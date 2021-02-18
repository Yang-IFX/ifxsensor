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

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>

#include "ifxRadar/SDK.h"
#include "util.h"

/*
==============================================================================
2. LOCAL DEFINITIONS
==============================================================================
*/

// A Description for these paramters can be found in ifx_Device_Config_t
// in DeviceConfig.h 
#define FREQUENCY_LOWER_HZ ((uint64_t)58600000000) // 58.6 GHZ
#define FREQUENCY_UPPER_HZ ((uint64_t)62900000000) // 62.9 GHz
#define NUM_SAMPLES_PER_CHIRP 128
#define NUM_CHIRPS_PER_FRAME 32
#define SAMPLE_RATE_HZ ((uint32_t)1000000) // 1 MHz
#define FRAME_REPETITION_TIME_S ((ifx_Float_t)0.2) // 0.2s

#define MAXIMUM_SPEED_MPS ((ifx_Float_t)2.5) // maximum speed in m/s that can be detected

#define ALPHA_MTI_FILTER ((ifx_Float_t)0.3)
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
static volatile bool g_is_running = true;

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

// Find indices (row, column) and value of maximum in a real matrix
static ifx_Float_t rdm_peak_search(const ifx_Matrix_R_t* matrix, uint32_t* rmax, uint32_t* cmax)
{
    *rmax = 0, *cmax = 0;
    ifx_Float_t max_value = IFX_MAT_AT(matrix, 0, 0);

    for (uint32_t r = 0; r < IFX_MAT_ROWS(matrix); r++)
    {
        for (uint32_t c = 0; c < IFX_MAT_COLS(matrix); c++)
        {
            ifx_Float_t value = IFX_MAT_AT(matrix, r, c);
            if (value > max_value)
            {
                *rmax = r;
                *cmax = c;
                max_value = value;
            }
        }
    }

    return max_value;
}

void signal_handler(int sig)
{
    if (sig == SIGINT)
        g_is_running = false;
}

int main(int argc, char** argv)
{
    ifx_Error_t ret = 0;
    ifx_Device_Handle_t device_handle = NULL;
    ifx_Frame_R_t* frame = NULL;
    ifx_Matrix_R_t* rdm = NULL;
    ifx_RDM_Handle_t rdm_handle = NULL;
    ifx_2DMTI_Handle_R_t mti_handle = NULL;

    printf("Radar SDK Version: %s\n", ifx_radar_sdk_get_version_string());
    signal(SIGINT, signal_handler);

    // Create the device handle
    device_handle = ifx_device_create();
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to open Device: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    // This is the device configuration. The parameters have been initialized with reasonable defaults for a
    // range-doppler example of approximately 2 meters of range with a range resolution of around 3.5cm.
    const ifx_Float_t center_frequency_Hz = (ifx_Float_t)(FREQUENCY_UPPER_HZ + FREQUENCY_LOWER_HZ) / 2;
    const ifx_Float_t wavelength_m = IFX_LIGHT_SPEED_MPS / center_frequency_Hz;
    const ifx_Device_Config_t device_config = {
        .num_samples_per_chirp = NUM_SAMPLES_PER_CHIRP,
        .num_chirps_per_frame = NUM_CHIRPS_PER_FRAME,
        .sample_rate_Hz = SAMPLE_RATE_HZ, // 1 MHz
        .frame_repetition_time_s = FRAME_REPETITION_TIME_S,
        .lower_frequency_Hz = FREQUENCY_LOWER_HZ,
        .upper_frequency_Hz = FREQUENCY_UPPER_HZ,
        .tx_power_level = 31,
        .rx_mask = 1, // one receiving antenna
        .chirp_repetition_time_s = wavelength_m / (4 * MAXIMUM_SPEED_MPS),
        .if_gain_dB = 33,
        .mimo_mode = IFX_MIMO_OFF
    };

    // Configure the device
    ifx_device_set_config(device_handle, &device_config);
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to initialize Device: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    // This helper function creates a frame which matches the device confiugration. 
    frame = ifx_device_create_frame_from_device_handle(device_handle);
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Error creating frame from device handle: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    // Size of FFT used for range and doppler. Increased FFT size for higher resolution.
    const uint32_t fft_size = 8 * device_config.num_samples_per_chirp;

    ifx_PPFFT_Config_t range_fft_config = {
        .fft_type = IFX_FFT_TYPE_R2C,
        .fft_size = fft_size,
        .mean_removal_enabled = true,
        .window_config = { IFX_WINDOW_BLACKMANHARRIS, device_config.num_samples_per_chirp, 0 },
        .is_normalized_window = 1
    };

    // length of one range bin
    const ifx_Float_t bandwidth_Hz = (ifx_Float_t)FREQUENCY_UPPER_HZ + (ifx_Float_t)FREQUENCY_LOWER_HZ;
    const ifx_Float_t range_bin_length = ((ifx_Float_t)IFX_LIGHT_SPEED_MPS) / (2 * bandwidth_Hz * range_fft_config.fft_size / device_config.num_samples_per_chirp);

    ifx_PPFFT_Config_t doppler_fft_config = {
        .fft_type = IFX_FFT_TYPE_C2C,
        .fft_size = fft_size,
        .mean_removal_enabled = true,
        .window_config = { IFX_WINDOW_CHEBYSHEV, device_config.num_chirps_per_frame, 100 },
        .is_normalized_window = 1
    };

    const ifx_Float_t speed_bin_length = (ifx_Float_t)(2 * MAXIMUM_SPEED_MPS / doppler_fft_config.fft_size);
    
    ifx_RDM_Config_t rdm_config = {
        .spect_threshold = (ifx_Float_t)1e-6,
        .output_scale_type = IFX_SCALE_TYPE_LINEAR,
        .range_fft_config = range_fft_config,
        .doppler_fft_config = doppler_fft_config
    };

    rdm_handle = ifx_rdm_create(&rdm_config);
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to create range doppler map handle: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    // In real to complex fft calculations the symmetric part of the result will be dropped.
    // The range fft will be applied in row direction therefore only half of fft size is expected. 
    rdm = ifx_mat_create_r(fft_size / 2, fft_size);
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to create range doppler matrix: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    // MTI Filter for static target cancellation
    mti_handle = ifx_2dmti_create_r(ALPHA_MTI_FILTER, IFX_MAT_ROWS(rdm), IFX_MAT_COLS(rdm));
    if ((ret = ifx_error_get()) != IFX_OK)
    {
        fprintf(stderr, "Failed to create 2dmti handle: %s (%x)\n", ifx_error_to_string(ret), ret);
        goto cleanup;
    }

    printf("# distance (in m), speed (in m/s)\n");

    while (g_is_running)
    {
        ret = ifx_device_get_next_frame(device_handle, frame);
        if (ret != IFX_OK)
        {
            fprintf(stderr, "Error getting next frame: %s (%x)\n", ifx_error_to_string(ret), ret);
            break;
        }

        ifx_rdm_run_r(rdm_handle, frame->rx_data[0], rdm);
        if ((ret = ifx_error_get()) != IFX_OK)
        {
            fprintf(stderr, "Failed to run range doppler map: %s (%x)\n", ifx_error_to_string(ret), ret);
            goto cleanup;
        }

        ifx_2dmti_run_r(mti_handle, rdm, rdm);

        // do peak search
        uint32_t rmax, cmax;
        const ifx_Float_t max_value = rdm_peak_search(rdm, &rmax, &cmax);
        // avoid compiler warning: we are ignoring max_value
        (void)max_value;

        // distance
        const ifx_Float_t distance = rmax * range_bin_length;
        // speed
        const ifx_Float_t speed = (((ifx_Float_t)IFX_MAT_COLS(rdm) / 2) - (ifx_Float_t)cmax) * speed_bin_length;

        printf("%g, %g\n", distance, speed);
    }

cleanup:

    ifx_device_destroy(device_handle);
    device_handle = NULL;
    ifx_frame_destroy_r(frame);
    ifx_rdm_destroy(rdm_handle);
    ifx_mat_destroy_r(rdm);
    ifx_2dmti_destroy_r(mti_handle);
    return EXIT_SUCCESS;
}

/**
* @}
*/