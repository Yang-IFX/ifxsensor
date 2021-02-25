/**
* \copyright
* MIT License
*
* Copyright (c) 2021 Infineon Technologies AG
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
* SOFTWARE.
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file ifx_bgt60_register_dump_publisher.cpp
*
*
*
* @{
*/

#include "ifx_bgt60_register_dump_publisher.h"

using namespace ifx;

Bgt60RegisterDumpPublisher::~Bgt60RegisterDumpPublisher()
{
    _running = false;
    if(_pub_thread.joinable())
        _pub_thread.join();
}

bool
Bgt60RegisterDumpPublisher::init(ros::NodeHandle& node,
                     const std::string& pub_topic_name,
                     const int pub_queue_size,
                     std::shared_ptr<RingBuffer<std::unique_ptr<
                          ifx_bgt60_device::ifx_bgt60_register_dump> > > buff)
{
    _buff = buff;
    _publisher = node.advertise<ifx_bgt60_device::ifx_bgt60_register_dump>(
                            pub_topic_name,
                            pub_queue_size);
    _running = true;

    _pub_thread = std::thread(&Bgt60RegisterDumpPublisher::publishData, this);

    ROS_DEBUG_STREAM("Initialize ifx::Bgt60RegisterDumpPublisher succeed with publish topic name: " <<
                        pub_topic_name << ", and message size: " << pub_queue_size);

    return _running;
}

void
Bgt60RegisterDumpPublisher::publishData()
{
    while(_running)
    {
        ROS_DEBUG_STREAM("ifx::Bgt60RegisterDumpPublisher waiting for dumped register data...");
        std::unique_ptr<ifx_bgt60_device::ifx_bgt60_register_dump> data = std::move(_buff->read());
        ifx_bgt60_device::ifx_bgt60_register_dump reg_data;
        reg_data.header.stamp = data->header.stamp;
        reg_data.header.frame_id = data->header.frame_id;
        reg_data.number = data->number;
        reg_data.values.swap(data->values);

        _publisher.publish(reg_data);
        ROS_DEBUG_STREAM("ifx::Bgt60RegisterDumpPublisher publish one message.");
    }
}

/**
* @}
*/