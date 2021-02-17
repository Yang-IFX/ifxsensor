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
* \file ifx_ring_buffer.h
*
* \brief   Define a template class of ring buffer SPECIALLY for 'ifx_bgt60_device'
*          use case. It is a very slim version implementation with minimum
*          functionality, and has following known limitations:
*            1. no capacity feature. As Publisher is continues publishing,
*                this won't casue problem for the moment.
*            2. always prefer to take std::unique_ptr as input.
*            3. no blocking, non-blocking, timeout etc. features for the moment.
*          The above points may be addressed step-by-step in the future as needed.
*
*
* @{
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
/**


* @}

*/
