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
/**
 * @file ifx_bgt60_device_handle.h
 *
 * @brief This file defines class RadarDeviceHandle to connect radar device
 *        and collect radar raw data into internal defined RingBuffer.
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
