/* ===========================================================================
** Copyright (c) 2020, Infineon Technologies AG All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** ===========================================================================
*/

#include <ros/console.h>

#include "ifx_bgt60_device_handle.h"

/* coming from radar_sdk/apps/c/common */
#include "json.h"


using namespace ifx;

RadarDeviceHandle::~RadarDeviceHandle()
{
    _running = false;
    if(_get_data_thread.joinable())
        _get_data_thread.join();

    if(_frame != nullptr)
        ifx_frame_destroy_r(_frame);

    if(_dev != nullptr)
        ifx_device_destroy(_dev);
}

bool
RadarDeviceHandle::init(const std::string& config_file,
                        const std::string& frame_id,
                        const std::string& uuid,
                        int ring_buff_size)
{
    ifx_Device_Config_t dev_config = { 0 };
    {
        ifx_json_t* json = ifx_json_create();
        if (!json)
        {
            ROS_ERROR_STREAM("Cannot create JSON structure");
            return false;
        }

        if (!config_file.empty())
        {
            /* read configuration from json file */
            bool ret = ifx_json_load_from_file(json, config_file.c_str());
            if (!ret)
            {
                ROS_ERROR_STREAM("Error parsing configuration file " << config_file << ": " << ifx_json_get_error(json));
                return false;
            }

            if (ifx_json_has_config_single_shape(json))
            {
                ret = ifx_json_get_device_config_single_shape(json, &dev_config);
                if (!ret)
                {
                    ROS_ERROR_STREAM("Error parsing fmcw_single_shape configuration: " << ifx_json_get_error(json));
                    return false;
                }
            }
            else if (ifx_json_has_config_scene(json))
            {
                ifx_Device_Metrics_t scene_config;
                ret = ifx_json_get_device_config_scene(json, &scene_config);
                if (!ret)
                {
                    ROS_ERROR_STREAM("Error parsing fmcw_scene configuration: " << ifx_json_get_error(json));
                    return false;
                }

                ifx_device_translate_metrics_to_config(NULL, &scene_config, &dev_config);
                if (ifx_error_get() != IFX_OK)
                {
                    ROS_ERROR_STREAM("Error converting scene to device configuration");
                    return false;
                }
            }
        }

        ifx_json_destroy(json);
    }

    _frame_id = frame_id;

    if(uuid.empty())
    {
        _dev = ifx_device_create();
    }
    else
    {
        _dev = ifx_device_create_by_uuid(reinterpret_cast<const uint8_t*>(uuid.c_str()));
        ROS_INFO_STREAM("Create device of uuid: " << uuid);
    }

    ifx_device_set_config(_dev, &dev_config);

    if(ifx_error_get() != IFX_OK)
    {
        ROS_ERROR_STREAM("Failed to initialize device with error: " <<
                         ifx_error_to_string(ifx_error_get()));
        ifx_device_destroy(_dev);
        return false;
    }

    _frame = ifx_device_create_frame_from_device_handle(_dev);
    if(_frame == nullptr)
    {
        ROS_ERROR_STREAM("Failed to create frame from device handle with error: " <<
                         ifx_error_to_string(ifx_error_get()));
        return false;
    }

    _buff = std::make_shared<RingBuffer<std::unique_ptr<ifx_bgt60_device::ifx_bgt60_raw_data> > >();

    _running = true;

    _get_data_thread = std::thread(&RadarDeviceHandle::getData, this);

    ROS_DEBUG_STREAM("RadarDeviceHandle::init succeed");
    return _running;
}

void
RadarDeviceHandle::getData()
{
    ROS_DEBUG_STREAM("RadarDeviceHandle::getData start");
    ifx_Error_t ret = IFX_OK;
    ifx_error_clear();
    while(_running)
    {
        ret = ifx_device_get_next_frame(_dev, _frame);
        if(ret == IFX_ERROR_FIFO_OVERFLOW)
        {
            ROS_WARN_STREAM("Radar device FIFO Overflow");
            continue;
        }
        else if(ret != IFX_OK)
        {
            ROS_ERROR_STREAM("Radar device unknown error");
            break;
        }

        /* create ifx_bgt60_device::ifx_bgt60_raw_data from ifx_Frame_t */
        auto radar_data = std::make_unique<ifx_bgt60_device::ifx_bgt60_raw_data>();

        ros::Time current_time = ros::Time::now();
        radar_data->header.stamp.sec = current_time.sec;
        radar_data->header.stamp.nsec = current_time.nsec;
        radar_data->header.frame_id = _frame_id;

        radar_data->num_sample_per_chirp = _frame->rx_data[0]->cols;
        radar_data->num_chirp_per_frame = _frame->rx_data[0]->rows;
        radar_data->num_rx = _frame->num_rx;
        radar_data->data.assign(radar_data->num_sample_per_chirp *
                                radar_data->num_chirp_per_frame *
                                radar_data->num_rx,
                                0);

        uint32_t idx = 0;
        for(uint8_t rx_idx = 0; rx_idx < _frame->num_rx; ++rx_idx)
        {
            ifx_Matrix_R_t* mat = _frame->rx_data[rx_idx];
            for(uint32_t row = 0; row < IFX_MAT_ROWS(mat); ++row)
            {
                for(uint32_t col = 0; col < IFX_MAT_COLS(mat); ++col)
                {
                    radar_data->data[idx++] = IFX_MAT_AT(mat, row, col);
                }
            }
        }

        _buff->write(std::move(radar_data));

        ROS_DEBUG_STREAM("getData: get one frame, push to buff");
    }
}


