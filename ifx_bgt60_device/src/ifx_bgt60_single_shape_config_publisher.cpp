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
* \file ifx_bgt60_single_shape_config_publisher.cpp
*
*
*
* @{
*/

#include "ifx_bgt60_single_shape_config_publisher.h"

using namespace ifx;

Bgt60SingleShapeConfigPublisher::~Bgt60SingleShapeConfigPublisher()
{
    _running = false;
    if(_pub_thread.joinable())
        _pub_thread.join();
}

bool
Bgt60SingleShapeConfigPublisher::init(ros::NodeHandle& node,
                     const std::string& pub_topic_name,
                     const int pub_queue_size,
                     std::shared_ptr<RingBuffer<std::unique_ptr<
                          ifx_bgt60_device::ifx_bgt60_single_shape_config> > > buff)
{
    _buff = buff;
    _publisher = node.advertise<ifx_bgt60_device::ifx_bgt60_single_shape_config>(
                            pub_topic_name,
                            pub_queue_size,
                            true);
    _running = true;

    _pub_thread = std::thread(&Bgt60SingleShapeConfigPublisher::publishData, this);

    ROS_DEBUG_STREAM("Initialize ifx::Bgt60SingleShapeConfigPublisher succeed with publish topic name: " <<
                        pub_topic_name << ", and message size: " << pub_queue_size);

    return _running;
}

void
Bgt60SingleShapeConfigPublisher::publishData()
{
    while(_running)
    {
        ROS_DEBUG_STREAM("ifx::Bgt60SingleShapeConfigPublisher waiting for config data...");
        std::unique_ptr<ifx_bgt60_device::ifx_bgt60_single_shape_config> data = std::move(_buff->read());
        ifx_bgt60_device::ifx_bgt60_single_shape_config cfg_data;
        cfg_data.header = data->header;
        cfg_data.device_type = data->device_type;
        cfg_data.device_uuid = data->device_uuid;
        cfg_data.single_shape = data->single_shape;

        _publisher.publish(cfg_data);
        ROS_DEBUG_STREAM("ifx::Bgt60SingleShapeConfigPublisher publish one message.");

        /**
         * 'config' message only publish once. It's latched.
         * It doesn't change during device running.
         * So thread can break here.
         */
        // break;
    }
}
