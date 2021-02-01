/* ===========================================================================
** Copyright (c) 2021, Infineon Technologies AG All rights reserved.
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
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** ===========================================================================
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