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
* \file main.cpp
*
* \brief   Top level main file to start 'ifx_bgt60_device' node.
*
*
* @{
*/


#include <ros/ros.h>

#include "ifx_bgt60_device/ifx_bgt60_device_node.h"


int main(int argc, char** argv)
{
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    // {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    /* Initialize ROS environment */
    ros::init(argc, argv, "ifx_bgt60_device");

    /**
     * Create and initialize ifx::Bgt60DeviceNode, which internally publishes
     * radar raw data to ROS environment
     */
    auto dev_node = ifx::Bgt60DeviceNode::createBgt60DeviceNode();
    if(!dev_node->init())
    {
        ROS_ERROR("ifx::Bgt60DeviceNode init() fail.");
        return -1;
    }

    /**
     * Check whether ROS environment
     */
    if(!ros::ok())
    {
        ROS_ERROR("ros initialization fail.");
        return -1;
    }

    /**
     * @brief Start the spin loop. 'Ctrl-C' will terminate it,
     *        as well as ROS nodes.
     */
    ros::spin();

    return 0;
}
