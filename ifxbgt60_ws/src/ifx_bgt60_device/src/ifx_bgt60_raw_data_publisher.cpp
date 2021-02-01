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

#include "ifx_bgt60_raw_data_publisher.h"

using namespace ifx;

RadarPublisher::~RadarPublisher()
{
    _running = false;
    if(_pub_thread.joinable())
        _pub_thread.join();
}

bool
RadarPublisher::init(ros::NodeHandle& node,
                     const std::string& pub_topic_name,
                     const int pub_queue_size,
                     std::shared_ptr<RingBuffer<std::unique_ptr<
                          ifx_bgt60_device::ifx_bgt60_raw_data> > > buff)
{
    _buff = buff;
    _publisher = node.advertise<ifx_bgt60_device::ifx_bgt60_raw_data>(
                            pub_topic_name,
                            pub_queue_size);
    _running = true;

    _pub_thread = std::thread(&RadarPublisher::publishData, this);

    ROS_DEBUG_STREAM("Initialize ifx::RadarPublisher succeed with publish topic name: " <<
                        pub_topic_name << ", and message size: " << pub_queue_size);

    return _running;
}

void
RadarPublisher::publishData()
{
    while(_running)
    {
        std::unique_ptr<ifx_bgt60_device::ifx_bgt60_raw_data> data = std::move(_buff->read());
        ifx_bgt60_device::ifx_bgt60_raw_data radar_data;

        radar_data.header.stamp = data->header.stamp;
        radar_data.header.frame_id = data->header.frame_id;
        radar_data.num_sample_per_chirp = data->num_sample_per_chirp;
        radar_data.num_chirp_per_frame = data->num_chirp_per_frame;
        radar_data.num_rx = data->num_rx;
        radar_data.data.swap(data->data);

        _publisher.publish(radar_data);
        ROS_DEBUG_STREAM("ifx::RadarPublisher publish one message.");
    }
}
