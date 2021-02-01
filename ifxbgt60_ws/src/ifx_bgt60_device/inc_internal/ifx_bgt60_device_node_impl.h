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
/**
 * @file ifx_bgt60_device_node_impl.h
 *
 * @brief The implementation of class Bgt60DeviceNode
 */

#ifndef IFX_BGT60_DEVICE_NODE_IMPL_H_
#define IFX_BGT60_DEVICE_NODE_IMPL_H_

#include <ros/ros.h>
#include <memory>

#include "ifx_bgt60_device/ifx_bgt60_device_node.h"

#include "ifx_bgt60_device_handle.h"
#include "ifx_bgt60_raw_data_publisher.h"


namespace ifx
{

class Bgt60DeviceNodeImpl: public Bgt60DeviceNode
{
public:
    /**
     * @brief Construct a new Radar Device Node Impl object
     */
    Bgt60DeviceNodeImpl() = default;
    /**
     * @brief Final implementation of init function
     *
     * @return true     Initialization succeed
     * @return false    Initialization fail
     */
    virtual bool init() final;
    /**
     * @brief Final implementation of getting initialization status
     */
    virtual inline bool isRunning() final { return _running; }

private:

    ros::NodeHandle _ros_node;
    bool            _running = {false};
    std::unique_ptr<RadarDeviceHandle>  _dev_handle = { nullptr };
    std::unique_ptr<RadarPublisher>     _publisher = { nullptr };
};

} // namespace ifx
#endif // IFX_BGT60_DEVICE_NODE_IMPL_H_
