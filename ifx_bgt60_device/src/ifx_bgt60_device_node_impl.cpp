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
* \file ifx_bgt60_device_node_impl.cpp
*
*
*
* @{
*/

#include "ifx_bgt60_device_node_impl.h"

using namespace ifx;

bool
Bgt60DeviceNodeImpl::init()
{
    std::string dev_config_file;
    std::string frame_id;
    std::string raw_data_pub_topic_name;
    int raw_data_pub_topic_size;
    std::string config_pub_topic_name;
    int config_pub_topic_size;
    std::string reg_dump_pub_topic_name;
    int reg_dump_pub_topic_size;
    bool dump_register;

    if(!_ros_node.getParam("ifx_bgt60_device_node/dev_config_file", dev_config_file))
    {
        ROS_ERROR_STREAM("Can't find \"dev_config_file\" defined in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/dump_register", dump_register))
    {
        /* uuid is reset to empty */
        dump_register = false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/frame_id", frame_id))
    {
        ROS_ERROR_STREAM("Undefined \"frame_id\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/raw_data_pub_topic_name", raw_data_pub_topic_name))
    {
        ROS_ERROR_STREAM("Undefined \"raw_data_pub_topic_name\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/raw_data_pub_topic_size", raw_data_pub_topic_size))
    {
        ROS_ERROR_STREAM("Undefined \"raw_data_pub_topic_size\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/config_pub_topic_name", config_pub_topic_name))
    {
        ROS_ERROR_STREAM("Undefined \"config_pub_topic_name\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/config_pub_topic_size", config_pub_topic_size))
    {
        ROS_ERROR_STREAM("Undefined \"config_pub_topic_size\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/reg_dump_pub_topic_name", reg_dump_pub_topic_name))
    {
        ROS_ERROR_STREAM("Undefined \"reg_dump_pub_topic_name\" in launch file.");
        return false;
    }

    if(!_ros_node.getParam("ifx_bgt60_device_node/reg_dump_pub_topic_size", reg_dump_pub_topic_size))
    {
        ROS_ERROR_STREAM("Undefined \"reg_dump_pub_topic_size\" in launch file.");
        return false;
    }

    _dev_handle = std::make_unique<ifx::RadarDeviceHandle>();
    if(!_dev_handle->init(dev_config_file, frame_id, dump_register))
    {
        ROS_ERROR_STREAM("Radar device initialization fail.");
        return false;
    }

    _raw_data_pub = std::make_unique<ifx::Bgt60RawDataPublisher>();
    if(!_raw_data_pub->init(_ros_node,
                            raw_data_pub_topic_name,
                            raw_data_pub_topic_size,
                            _dev_handle->accessBuffRawData()))
    {
        ROS_ERROR_STREAM("raw data publisher initialization fail.");
        return false;
    }

    _single_shape_config_pub =
            std::make_unique<ifx::Bgt60SingleShapeConfigPublisher>();
    if(!_single_shape_config_pub->init(_ros_node,
                            config_pub_topic_name,
                            config_pub_topic_size,
                            _dev_handle->accessBuffConfig()))
    {
        ROS_ERROR_STREAM("config publisher initialization fail.");
        return false;
    }

    if(dump_register)
    {
        _reg_dump_pub = std::make_unique<ifx::Bgt60RegisterDumpPublisher>();
        if(!_reg_dump_pub->init(_ros_node,
                                reg_dump_pub_topic_name,
                                reg_dump_pub_topic_size,
                                _dev_handle->accessBuffReg()))
        {
            ROS_ERROR_STREAM("register dump publisher initialization fail.");
            return false;
        }
    }

    ROS_DEBUG_STREAM("Bgt60DeviceNode::init succeed");
    _running = true;

    return _running;
}
/**


* @}

*/