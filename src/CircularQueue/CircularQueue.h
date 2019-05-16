/*
CircularQueue.h - Implementation of a lock-free circular queue for EspSoftwareSerial.
Copyright (c) 2019 Dirk O. Kaar. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef Circular_Queue_h
#define Circular_Queue_h

#include <atomic>
#include <memory>
#if !defined(ESP8266)
#include <mutex>
#endif

#if !defined(ESP8266) && !defined(ESP32)
#define ICACHE_RAM_ATTR
#endif

template< typename T > class CircularQueue
{
public:
    CircularQueue() = delete;
    CircularQueue(size_t capacity) : m_bufSize(capacity + 1), m_buffer(new std::atomic<T>[m_bufSize])
    {
        m_inPosT.store(0);
        m_outPos.store(0);
    }
    ~CircularQueue()
    {
        m_buffer.reset();
    }
    CircularQueue(const CircularQueue&) = delete;
    CircularQueue& operator=(const CircularQueue&) = delete;

    void flush() {
        m_outPos.store(m_inPosT.load());
    }

    size_t available()
    {
        int avail = static_cast<int>(m_inPosT.load() - m_outPos.load());
        if (avail < 0) avail += m_bufSize;
        return avail;
    }

    size_t availableForWrite()
    {
        int avail = static_cast<int>(m_outPos.load() - m_inPosT.load()) - 1;
        if (avail < 0) avail += m_bufSize;
        return avail;
    }

    T peek()
    {
        auto outPos = m_outPos.load();
        return (m_inPosT.load() == outPos) ? defaultValue : m_buffer[outPos].load();
    }

    bool ICACHE_RAM_ATTR push(T val)
    {
        auto inPos = m_inPosT.load();
        int next = (inPos + 1) % m_bufSize;
        if (next == m_outPos.load()) {
            return false;
        }

        m_buffer[inPos].store(val);

        m_inPosT.store(next);
        return true;
    }

    T pop()
    {
        auto outPos = m_outPos.load();
        if (m_inPosT.load() == outPos) return defaultValue;
        auto val = m_buffer[outPos].load();
        m_outPos.store((outPos + 1) % m_bufSize);
        return val;
    }

    size_t pop_n(T* buffer, size_t size) {
        size_t avail = size = std::min(size, available());
        if (!avail) return 0;
        auto outPos = m_outPos.load();
        size_t n = std::min(avail, m_bufSize - outPos);
        buffer = std::copy_n(m_buffer.get() + outPos, n, buffer);
        avail -= n;
        if (0 < avail) {
            buffer = std::copy_n(m_buffer.get(), avail, buffer);
        }
        m_outPos.store((outPos + size) % m_bufSize);
        return size;
    }

protected:
    const T defaultValue = {};
    unsigned m_bufSize;
    std::unique_ptr<std::atomic<T>[] > m_buffer;
    std::atomic<unsigned> m_inPosT;
    std::atomic<unsigned> m_outPos;
};

template< typename T > class CircularQueueMP : protected CircularQueue<T>
{
public:
    CircularQueueMP(size_t capacity) : CircularQueue<T>(capacity)
    {
    }
    using CircularQueue<T>::flush;
    using CircularQueue<T>::available;
    using CircularQueue<T>::availableForWrite;
    using CircularQueue<T>::peek;
    using CircularQueue<T>::pop;
    using CircularQueue<T>::pop_n;

    bool ICACHE_RAM_ATTR push(T val)
#ifdef ESP8266
    {
        uint32_t savedPS = xt_rsil(15);
        auto inPos = CircularQueue<T>::m_inPosT.load();
        unsigned next = (inPos + 1) % CircularQueue<T>::m_bufSize;
        if (next == CircularQueue<T>::m_outPos.load()) {
            xt_wsr_ps(savedPS);
            return false;
        }

        CircularQueue<T>::m_buffer[inPos].store(val);

        CircularQueue<T>::m_inPosT.store(next);

        xt_wsr_ps(savedPS);
        return true;
    }
#else
    {
        std::lock_guard<std::mutex> lock(m_pushMtx);
        auto inPos = CircularQueue<T>::m_inPosT.load();
        unsigned next = (inPos + 1) % CircularQueue<T>::m_bufSize;
        if (next == CircularQueue<T>::m_outPos.load()) {
            return false;
        }

        CircularQueue<T>::m_buffer[inPos].store(val);

        CircularQueue<T>::m_inPosT.store(next);
        return true;
    }

protected:
    std::mutex m_pushMtx;
#endif
};

#endif // Circular_Queue_h
