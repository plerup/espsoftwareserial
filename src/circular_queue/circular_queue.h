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

#ifndef __circular_queue_h
#define __circular_queue_h

#include <atomic>
#include <memory>
#include <algorithm>
#if !defined(ESP8266)
#include <mutex>
#endif

#if !defined(ESP8266) && !defined(ESP32)
#define ICACHE_RAM_ATTR
#endif

template< typename T > class circular_queue
{
public:
	circular_queue() : m_bufSize(0)
	{
		m_inPosT.store(0);
		m_outPos.store(0);
	}
	circular_queue(const size_t capacity) : m_bufSize(capacity + 1), m_buffer(new std::atomic<T>[m_bufSize])
	{
		m_inPosT.store(0);
		m_outPos.store(0);
	}
	~circular_queue()
	{
		m_buffer.reset();
	}
	circular_queue(const circular_queue&) = delete;
	circular_queue& operator=(const circular_queue&) = delete;

	void capacity(const size_t cap)
	{
		m_bufSize = cap + 1;
		m_buffer.reset(new std::atomic<T>[m_bufSize]);
		m_inPosT.store(0);
		m_outPos.store(0);
	}

	void flush()
	{
		m_outPos.store(m_inPosT.load());
	}

	size_t available() const
	{
		int avail = static_cast<int>(m_inPosT.load() - m_outPos.load());
		if (avail < 0) avail += m_bufSize;
		return avail;
	}

	size_t available_for_push() const
	{
		int avail = static_cast<int>(m_outPos.load() - m_inPosT.load()) - 1;
		if (avail < 0) avail += m_bufSize;
		return avail;
	}

	T peek() const
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

	size_t push_n(T * buffer, size_t size)
	{
		auto inPos = m_inPosT.load();
		auto outPos = m_outPos.load();

		size_t blockSize = (outPos > inPos) ? outPos - 1 - inPos : (outPos == 0) ? m_bufSize - 1 - inPos : m_bufSize - inPos;
		blockSize = std::min(size, blockSize);
		if (!blockSize) return 0;
		int next = (inPos + blockSize) % m_bufSize;
		auto dest = m_buffer.get() + inPos;
		std::copy_n(buffer, blockSize, dest);
		size = std::min(size - blockSize, outPos > 1 ? outPos - 1 : 0);
		next += size;
		std::copy_n(buffer + blockSize, size, m_buffer.get());
		m_inPosT.store(next);
		return blockSize + size;
	}

	T pop()
	{
		auto outPos = m_outPos.load();
		if (m_inPosT.load() == outPos) return defaultValue;
		auto val = m_buffer[outPos].load();
		m_outPos.store((outPos + 1) % m_bufSize);
		return val;
	}

	size_t pop_n(T * buffer, size_t size) {
		size_t avail = size = std::min(size, available());
		if (!avail) return 0;
		auto outPos = m_outPos.load();
		size_t n = std::min(avail, m_bufSize - outPos);
		buffer = std::copy_n(m_buffer.get() + outPos, n, buffer);
		avail -= n;
		std::copy_n(m_buffer.get(), avail, buffer);
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

template< typename T > class circular_queue_mp : protected circular_queue<T>
{
public:
	circular_queue_mp() = default;
	circular_queue_mp(size_t capacity) : circular_queue<T>(capacity)
	{
	}
	using circular_queue<T>::capacity;
	using circular_queue<T>::flush;
	using circular_queue<T>::available;
	using circular_queue<T>::available_for_push;
	using circular_queue<T>::peek;
	using circular_queue<T>::pop;
	using circular_queue<T>::pop_n;

	bool ICACHE_RAM_ATTR push(T val)
#ifdef ESP8266
	{
		uint32_t savedPS = xt_rsil(15);
		auto res = CircularQueue<T>::push(val);
		xt_wsr_ps(savedPS);
		return res;
	}
#else
	{
		std::lock_guard<std::mutex> lock(m_pushMtx);
		return circular_queue<T>::push(val);
	}
#endif

	size_t push_n(T* buffer, size_t size)
#ifdef ESP8266
	{
		uint32_t savedPS = xt_rsil(15);
		auto res = CircularQueue<T>::push_n(buffer, size);
		xt_wsr_ps(savedPS);
		return res;
	}
#else
	{
		std::lock_guard<std::mutex> lock(m_pushMtx);
		return circular_queue<T>::push_n(buffer, size);
	}

protected:
	std::mutex m_pushMtx;
#endif
};

#endif // __circular_queue_h
