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
* \file ifx_bgt60_device_handle.cpp
*
*
*
* @{
*/


#include <ros/console.h>

#include "ifx_bgt60_device_handle.h"

/* coming from radar_sdk/apps/c/common */
#include "json.hpp"


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
                        bool dump_register,
                        int ring_buff_size)
{
    _frame_id = frame_id;

    ifx_Device_Config_t dev_config = { 0 };
    auto single_shape_config =
            std::make_unique<ifx_bgt60_device::ifx_bgt60_single_shape_config>();

    ifxJsonConfiguration json = ifxJsonConfiguration();
    try
    {
        json.load_from_file(config_file);
    }
    catch (const std::string& reason)
    {
        ROS_ERROR_STREAM("Error loading json file " << config_file << ": " << reason);
        return false;
    }

    // open device
    if (json.has_device())
    {
        for (auto& uuid : json.get_device_uuids())
        {
            _dev = ifx_device_create_by_uuid(uuid.data());

            // we found the device
            if (_dev)
            {
                char suuid[36];
                ifx_util_uuid_to_string(uuid.data(), suuid);
                single_shape_config->device_uuid = std::string(suuid);
                break;
            }
        }

        if (!_dev)
        {
            ROS_ERROR_STREAM("Could not find device with matching uuid.");
            return false;
        }
    }
    else
    {
        _dev = ifx_device_create();

        if (!_dev)
        {
            ROS_ERROR_STREAM("Could not find any device.");
            return false;
        }
    }

    // get configuration
    if (json.has_config_fmcw_scene())
    {
        ifx_Device_Metrics_t metrics = {};
        json.get_config_fmcw_scene(&metrics);

        ifx_device_translate_metrics_to_config(_dev, &metrics, &dev_config);
    }
    else if (json.has_config_fmcw_single_shape())
    {
        json.get_config_fmcw_single_shape(&dev_config);
    }
    else
    {
        ROS_ERROR_STREAM("No device configuration in json file.");
        ifx_device_destroy(_dev);
        return false;
    }

    ros::Time current_time = ros::Time::now();
    single_shape_config->header.stamp.sec = current_time.sec;
    single_shape_config->header.stamp.nsec = current_time.nsec;
    single_shape_config->header.frame_id = frame_id;

    /* `device_type` reserved for future */
    // single_shape_config->device_type = "BGT60ATR24";

    single_shape_config->single_shape.rx_antennas_mask = dev_config.rx_mask;
    single_shape_config->single_shape.tx_antennas_mask = dev_config.tx_mask;
    single_shape_config->single_shape.tx_power_level = dev_config.tx_power_level;
    single_shape_config->single_shape.if_gain_dB = dev_config.if_gain_dB;
    single_shape_config->single_shape.lower_frequency_Hz = dev_config.lower_frequency_Hz;
    single_shape_config->single_shape.upper_frequency_Hz = dev_config.upper_frequency_Hz;
    single_shape_config->single_shape.num_chirps_per_frame = dev_config.num_chirps_per_frame;
    single_shape_config->single_shape.num_samples_per_chirp = dev_config.num_samples_per_chirp;
    single_shape_config->single_shape.chirp_repetition_time_s = dev_config.chirp_repetition_time_s;
    single_shape_config->single_shape.frame_repetition_time_s = dev_config.frame_repetition_time_s;


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

    _buff_raw = std::make_shared<RingBuffer<std::unique_ptr<ifx_bgt60_device::ifx_bgt60_raw_data> > >();
    _buff_cfg = std::make_shared<RingBuffer<std::unique_ptr<ifx_bgt60_device::ifx_bgt60_single_shape_config> > >();
    _dump_register = dump_register;
    if(_dump_register)
    {
        _buff_reg = std::make_shared<RingBuffer<std::unique_ptr<ifx_bgt60_device::ifx_bgt60_register_dump> > >();
    }
    _running = true;

    _buff_cfg->write(std::move(single_shape_config));

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
        /**
         * @brief Step 1: get raw data
         *
         */
        ROS_DEBUG_STREAM("RadarDeviceHandle::getData get_next_frame");
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

        ROS_DEBUG_STREAM("RadarDeviceHandle::getData moving data to buffer");
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

        _buff_raw->write(std::move(radar_data));

        ROS_DEBUG_STREAM("RadarDeviceHandle::getData push to buff done");

        /**
         * @brief Step 2 (optional): get dumped register value
         *
         */
        if(_dump_register)
        {
            ROS_DEBUG_STREAM("RadarDeviceHandle::getData get_dumped_register");
            uint32_t registers[256];
            uint8_t num_register = 0;
            ret = ifx_device_get_dumped_registers(_dev, registers, &num_register);

            auto register_dump = std::make_unique<ifx_bgt60_device::ifx_bgt60_register_dump>();
            register_dump->header.stamp.sec = current_time.sec;
            register_dump->header.stamp.nsec = current_time.nsec;
            register_dump->header.frame_id = _frame_id;
            register_dump->number = num_register;
            register_dump->values.assign(num_register, 0);
            for(uint8_t num = 0; num < num_register; ++num)
            {
                register_dump->values[num] = registers[num];
            }

            _buff_reg->write(std::move(register_dump));
        }
    }
}
/**


* @}

*/

