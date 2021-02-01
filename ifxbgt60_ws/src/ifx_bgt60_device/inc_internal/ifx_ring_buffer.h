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
 * @file ifx_ring_buffer.h
 *
 * @brief Define a template class of ring buffer SPECIALLY for 'ifx_bgt60_device'
 *        use case. It is a very slim version implementation with minimum
 *        functionality, and has following known limitations:
 *          1. no capacity feature. As Publisher is continues publishing,
 *              this won't casue problem for the moment.
 *          2. always prefer to take std::unique_ptr as input.
 *          3. no blocking, non-blocking, timeout etc. features for the moment.
 *        The above points may be addressed step-by-step in the future as needed.
 */


#ifndef IFX_RING_BUFFER_H_
#define IFX_RING_BUFFER_H_

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace ifx
{

template <typename T>
class RingBuffer
{
public:
    /**
     * @brief Clean up ring buffer
     */
    void clean()
    {
        std::unique_lock<std::mutex> locker(_mutex);
        _buffer.clear();
        locker.unlock();
        _condv.notify_one();
        return;
    }
    /**
     * @brief Write one element into ring buffer
     *
     * @param input An input element of std::unique_ptr type.
     */
    void write(T input)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        _buffer.push_back(std::move(input));
        locker.unlock();
        _condv.notify_one();
        return;
    }
    /**
     * @brief Read out one element from ring buffer
     *
     * @return T    Return one element out.
     */
    T read()
    {
        std::unique_lock<std::mutex> locker(_mutex);
        _condv.wait(locker, [this](){return _buffer.size() > 0;});
        T front = std::move(_buffer.front());
        _buffer.pop_front();
        locker.unlock();
        _condv.notify_all();
        return front;
    }
    /**
     * @brief Get the number of elements inside current ring buffer
     *
     * @return int  Number of elements.
     */
    int size()
    {
        std::unique_lock<std::mutex> locker(_mutex);
        int size = _buffer.size();
        locker.unlock();
        return size;
    }

private:
    std::condition_variable _condv;
    std::mutex              _mutex;
    std::deque<T>           _buffer;
};

} // namespace ifx
#endif // IFX_RING_BUFFER_H_
