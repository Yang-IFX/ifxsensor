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
* \file ifx_bgt60_device_node.h
*
* \brief   Interface class Bgt60DeviceNode
*
*
* @{
*/


#ifndef IFX_BGT60_DEVICE_NODE_H_
#define IFX_BGT60_DEVICE_NODE_H_

#include <ros/ros.h>
#include <memory>


namespace ifx
{

class Bgt60DeviceNode
{
public:
    /**
     * @brief Destroy the Radar Device Node object
     */
    virtual ~Bgt60DeviceNode() = default;
    /**
     * @brief Interface wrapper function to create a Radar Device Node object
     *
     * @return std::unique_ptr<Bgt60DeviceNode>
     */
    static std::unique_ptr<Bgt60DeviceNode> createBgt60DeviceNode();

    virtual bool init() = 0;
    virtual inline bool isRunning() = 0;
};

} // namespace ifx
#endif // IFX_BGT60_DEVICE_NODE_H_

/**
* @}
*/
