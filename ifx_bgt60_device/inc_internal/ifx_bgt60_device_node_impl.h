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
* \file ifx_bgt60_device_node_impl.h
*
* \brief   The implementation of class Bgt60DeviceNode
*
*
* @{
*/


#ifndef IFX_BGT60_DEVICE_NODE_IMPL_H_
#define IFX_BGT60_DEVICE_NODE_IMPL_H_

#include <ros/ros.h>
#include <memory>

#include "ifx_bgt60_device/ifx_bgt60_device_node.h"

#include "ifx_bgt60_device_handle.h"
#include "ifx_bgt60_raw_data_publisher.h"
#include "ifx_bgt60_single_shape_config_publisher.h"
#include "ifx_bgt60_register_dump_publisher.h"


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
    std::unique_ptr<RadarDeviceHandle>          _dev_handle = { nullptr };
    std::unique_ptr<Bgt60RawDataPublisher>      _raw_data_pub = { nullptr };
    std::unique_ptr<Bgt60SingleShapeConfigPublisher>
                                    _single_shape_config_pub = { nullptr };
    std::unique_ptr<Bgt60RegisterDumpPublisher> _reg_dump_pub = { nullptr };
};

} // namespace ifx
#endif // IFX_BGT60_DEVICE_NODE_IMPL_H_
