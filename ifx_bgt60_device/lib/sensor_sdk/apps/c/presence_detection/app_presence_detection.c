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
* \file app_presence_detection.c
*
* \brief   Presence detection application main program source file.
*
*
* @{
*/



/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <math.h>
#include <string.h>

#include "ifxRadar/SDK.h"

#include "presence_sensing_defaults.h"

//#include "json_presence_sensing_config.h"
#include "app_common.h"
#include "json.h"

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/
#define RX_ANTENNA_ID   0   // This implementation uses only 1 Rx Antenna

#define WAVFILE_PRESENCE "share/Hello_fast.wav"
#define WAVFILE_ABSENCE  "share/Goodbye_fast.wav"

/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
*/
typedef struct {
    ifx_PresenceSensing_Handle_t* presencesensing;
} presence_t;

/*
==============================================================================
   4. LOCAL DATA
==============================================================================
*/
static ifx_Device_Metrics_t default_metrics =
{
    .sample_rate_Hz = IFX_ADC_SAMPLERATE_HZ,
    .rx_mask = IFX_RX_MASK,
    .tx_mask = IFX_TX_MASK,
    .tx_power_level = IFX_BGT_TX_POWER,
    .if_gain_dB = IFX_IF_GAIN_DB,
    .range_resolution_m = IFX_RANGE_RESOLUTION_M,
    .max_range_m = IFX_MAX_RANGE_M,
    .speed_resolution_m_s = IFX_SPEED_RESOLUTION_M_S,
    .max_speed_m_s = IFX_MAX_SPEED_M_S,
    .frame_repetition_time_s = ((ifx_Float_t)1)/IFX_FRAME_RATE_HZ
};

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

/**
 * @brief Callback function to play a media when status changes.
 *
 * @param new_state The new state.
 */
static void state_change_callback(ifx_PresenceSensing_State_t new_state, void* context)
{
    (void)context;
    if (new_state == IFX_PRESENCE_SENSING_PRESENT)
        app_playaudio(WAVFILE_PRESENCE);
    else //if(new_state == ABSENCE)
        app_playaudio(WAVFILE_ABSENCE);
}

//----------------------------------------------------------------------------

ifx_Error_t presence_init(presence_t* presence_context)
{
    presence_context->presencesensing = NULL;

    return IFX_OK;
}

ifx_Error_t presence_config(presence_t* presence_context, ifx_json_t* json, ifx_Device_Config_t* dev_config)
{
    ifx_PresenceSensing_Config_t ps_config = { 0 };

    // check for current limitation of presence sensing application
    if (dev_config->rx_mask != IFX_RX_MASK)
    {
        fprintf(stderr, "Presence sensing currently only works with RX antenna 3 activated (rx_mask=4)");
        return IFX_ERROR_APP;
    }

    if (ifx_json_has_presence_sensing(json))
    {
        bool ret = ifx_json_get_presence_sensing(json, dev_config, &ps_config);
        if (!ret)
        {
            fprintf(stderr, "Error parsing presence sensing configuration: %s\n", ifx_json_get_error(json));
            return IFX_ERROR_APP;
        }
    }
    else
        ifx_presence_sensing_get_config_defaults(dev_config, &ps_config);

    ps_config.state_change_cb = state_change_callback;

    presence_context->presencesensing = ifx_presence_sensing_create(&ps_config);

    return ifx_error_get();
}


ifx_Error_t presence_cleanup(presence_t* presence_context)
{
    ifx_presence_sensing_destroy(presence_context->presencesensing);

    return ifx_error_get();
}

ifx_Error_t presence_process(presence_t* presence_context, ifx_Frame_R_t* frame)
{
    ifx_PresenceSensing_Result_t ps_result = { 0 };

    ifx_Matrix_R_t* rx_data = ifx_frame_get_mat_from_antenna_r(frame, RX_ANTENNA_ID);
    ifx_presence_sensing_run(presence_context->presencesensing, rx_data, &ps_result);
    if (ifx_error_get() != IFX_OK)
        return ifx_error_get();

    if (ps_result.cur_presence_state == IFX_PRESENCE_SENSING_PRESENT)
        app_print(", \"is_present\": true");
    else //if(cur_state == ABSENCE)
        app_print(", \"is_present\": false");

    if (!isnan(ps_result.target_distance_m) && !isnan(ps_result.target_speed_m_s))
    {
        app_verbose(", \"target_distance_m\": %g, \"target_speed_m_s\": %g, \"target_signal_strength\": %g",
            ps_result.target_distance_m, ps_result.target_speed_m_s, ps_result.target_signal_strength);
    }

    return ifx_error_get();
}

/*
==============================================================================
   7. MAIN METHOD
==============================================================================
 */

int main(int argc, char* argv[])
{
    app_t s_presence;
    presence_t presence_context;
    int exitcode;

    // function Description
    static const char* app_description = "Presence detection";
    static const char* app_epilog = NULL;

    s_presence.app_description = app_description;
    s_presence.app_epilog = app_epilog;

    s_presence.app_init = (void *)&presence_init;
    s_presence.app_config = (void*)&presence_config;
    s_presence.app_process = (void*)&presence_process; // process rename to
    s_presence.app_cleanup = (void*)&presence_cleanup;

    s_presence.default_metrics = &default_metrics;

    exitcode = app_start(argc, argv, &s_presence, &presence_context);

    return exitcode;
}

/**
* @}
*/