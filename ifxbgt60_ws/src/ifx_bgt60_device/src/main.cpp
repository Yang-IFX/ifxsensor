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
 * @file main.cpp
 *
 * @brief Top level main file to start 'ifx_bgt60_device' node.
 */

#include <ros/ros.h>

#include "ifx_bgt60_device/ifx_bgt60_device_node.h"


int main(int argc, char** argv)
{
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
