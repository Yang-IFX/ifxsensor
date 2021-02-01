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
