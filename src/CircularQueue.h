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

template< typename T > class CircularQueue
{
public:
	CircularQueue() = delete;
	CircularQueue(size_t capacity) : m_bufSize(capacity + 1), m_buffer(new std::atomic<T>[m_bufSize])
	{
		m_inPos.store(0);
		m_outPos.store(0);
	}
	~CircularQueue()
	{
		m_buffer.reset();
	}
	CircularQueue(const CircularQueue&) = delete;
	CircularQueue& operator=(const CircularQueue&) = delete;

	void flush() {
		m_outPos.store(m_inPos.load());
	}

	size_t available()
	{
		ssize_t avail = m_inPos.load() - m_outPos.load();
		if (avail < 0) avail += m_bufSize;
		return avail;
	}

	T peek()
	{
		auto outPos = m_outPos.load();
		return (m_inPos.load() == outPos) ? defaultValue : m_buffer[outPos].load();
	}

	bool ICACHE_RAM_ATTR push(T val)
	{
		auto inPos = m_inPos.load();
		int next = (inPos + 1) % m_bufSize;
		if (next != m_outPos.load()) {
			m_buffer[inPos].store(val);
			m_inPos.store(next);
			return true;
		}
		return false;
	}

	T pop()
	{
		auto outPos = m_outPos.load();
		if (m_inPos.load() == outPos) return defaultValue;
		auto val = m_buffer[outPos].load();
		m_outPos.store((outPos + 1) % m_bufSize);
		return val;
	}

	size_t pop_n(T* buffer, size_t size) {
		size_t avail = size = min(size, available());
		if (!avail) return 0;
		auto outPos = m_outPos.load();
		size_t n = min(avail, static_cast<size_t>(m_bufSize - outPos));
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
	int m_bufSize;
	std::unique_ptr<std::atomic<T>[] > m_buffer;
	std::atomic<int> m_inPos;
	std::atomic<int> m_outPos;
};

#endif // Circular_Queue_h
