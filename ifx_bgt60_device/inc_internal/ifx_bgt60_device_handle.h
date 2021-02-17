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
* \file ifx_bgt60_device_handle.h
*
* \brief   This file defines class RadarDeviceHandle to connect radar device
*          and collect radar raw data into internal defined RingBuffer.
*
*
* @{
*/

#ifndef IFX_BGT60_DEVICE_HANDLE_H_
#define IFX_BGT60_DEVICE_HANDLE_H_


#include <thread>
#include <memory>

/* Generated message header file */
#include <ifx_bgt60_device/ifx_bgt60_raw_data.h>
#include <ifx_bgt60_device/ifx_bgt60_single_shape_config.h>
#include <ifx_bgt60_device/ifx_bgt60_register_dump.h>

#include "ifx_ring_buffer.h"
#include "ifxRadar/SDK.h"


namespace ifx
{

class RadarDeviceHandle
{
public:
    /**
     * @brief Construct a new RadarDeviceHandle object
     */
    RadarDeviceHandle() = default;
    /**
     * @brief Destroy the RadarDeviceHandle object
     */
    ~RadarDeviceHandle();
    /**
     * @brief Initialize RadarDeviceHandle object
     *
     * @param config_file       File path for device configuration.
     * @param frame_id          Id to identify different message, when playing back.
     * @param ring_buff_size    Size of ring buffer member. Reserved for future use.
     * @return true             Initialization succeed
     * @return false            Initialization fail
     */
    bool init(const std::string& config_file,
              const std::string& frame_id,
              bool dump_register = false,
              int ring_buff_size = 64);
    /**
     * @brief Access raw data buffer
     *
     * @return std::shared_ptr<RingBuffer<std::unique_ptr<
     * ifx_bgt60_device::ifx_bgt60_raw_data> > >
     */
    inline std::shared_ptr<RingBuffer<std::unique_ptr<
                ifx_bgt60_device::ifx_bgt60_raw_data> > > accessBuffRawData() { return _buff_raw; }

    /**
     * @brief Access radar config buffer
     *
     * @return std::shared_ptr<RingBuffer<std::unique_ptr<
     * ifx_bgt60_device::ifx_bgt60_single_shape_config> > >
     */
    inline std::shared_ptr<RingBuffer<std::unique_ptr<
                ifx_bgt60_device::ifx_bgt60_single_shape_config> > > accessBuffConfig() { return _buff_cfg; }

    /**
     * @brief Access dumped register buffer
     *
     * @return std::shared_ptr<RingBuffer<std::unique_ptr<
     * ifx_bgt60_device::ifx_bgt60_register_dump> > >
     */
    inline std::shared_ptr<RingBuffer<std::unique_ptr<
                ifx_bgt60_device::ifx_bgt60_register_dump> > > accessBuffReg() { return _buff_reg; }

private:
    /**
     * @brief Get radar raw data, and save it to ring buffer in a
     *        separate thread.
     */
    void getData();

    /**
     * "frame_id" is used to save descriptions about current running messages.
     * When playing back message, it's easy to differentiate messages.
     */
    std::string     _frame_id;

    std::shared_ptr<RingBuffer<std::unique_ptr<
        ifx_bgt60_device::ifx_bgt60_raw_data> > > _buff_raw = { nullptr };

    std::shared_ptr<RingBuffer<std::unique_ptr<
        ifx_bgt60_device::ifx_bgt60_single_shape_config> > > _buff_cfg = { nullptr };

    std::shared_ptr<RingBuffer<std::unique_ptr<
        ifx_bgt60_device::ifx_bgt60_register_dump> > > _buff_reg = { nullptr };

    ifx_Device_Handle_t _dev = { nullptr };
    ifx_Frame_R_t*      _frame = { nullptr };
    std::thread         _get_data_thread;
    bool                _running = { false };
    bool                _dump_register = { false };
};

} // namespace ifx
#endif // IFX_BGT60_DEVICE_HANDLE_H_
