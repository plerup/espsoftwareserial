/*
circular_queue.h - Implementation of a lock-free circular queue for EspSoftwareSerial.
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
#ifdef ESP8266
#include "interrupts.h"
#else
#include <mutex>
#endif

#if !defined(ESP8266) && !defined(ESP32)
#define ICACHE_RAM_ATTR
#define IRAM_ATTR
#endif

template< typename T > class circular_queue
{
public:
	circular_queue() : m_bufSize(1)
	{
		m_inPos.store(0);
		m_outPos.store(0);
	}
	circular_queue(const size_t capacity) : m_bufSize(capacity + 1), m_buffer(new T[m_bufSize])
	{
		m_inPos.store(0);
		m_outPos.store(0);
	}
	circular_queue(circular_queue&& cq) :
		m_bufSize(cq.m_bufSize), m_buffer(cq.m_buffer), m_inPos(cq.m_inPos.load()), m_outPos(cq.m_outPos.load())
	{}
	~circular_queue()
	{
		m_buffer.reset();
	}
	circular_queue(const circular_queue&) = delete;
	circular_queue& operator=(circular_queue&& cq)
	{
		m_bufSize = cq.m_bufSize;
		m_buffer = cq.m_buffer;
		m_inPos.store(cq.m_inPos.load());
		m_outPos.store(cq.m_outPos.load());
	}
	circular_queue& operator=(const circular_queue&) = delete;

	size_t capacity() const
	{
		return m_bufSize - 1;
	}

	bool capacity(const size_t cap)
	{
		if (cap + 1 == m_bufSize) return true;
		else if (available() > cap) return false;
		std::unique_ptr<T[] > buffer(new T[cap + 1]);
		const auto available = pop_n(buffer, cap);
		m_buffer.reset(buffer);
		m_bufSize = cap + 1;
		std::atomic_thread_fence(std::memory_order_release);
		m_inPos.store(available, std::memory_order_relaxed);
		m_outPos.store(0, std::memory_order_relaxed);
		return true;
	}

	void flush()
	{
		m_outPos.store(m_inPos.load());
	}

	size_t available() const
	{
		int avail = static_cast<int>(m_inPos.load() - m_outPos.load());
		if (avail < 0) avail += m_bufSize;
		return avail;
	}

	size_t available_for_push() const
	{
		int avail = static_cast<int>(m_outPos.load() - m_inPos.load()) - 1;
		if (avail < 0) avail += m_bufSize;
		return avail;
	}

	T peek() const
	{
		const auto outPos = m_outPos.load(std::memory_order_relaxed);
		const auto inPos = m_inPos.load(std::memory_order_relaxed);
		std::atomic_thread_fence(std::memory_order_acquire);
		if (inPos == outPos) return defaultValue;
		else return m_buffer[outPos];
	}

	bool ICACHE_RAM_ATTR push(T&& val)
	{
		const auto inPos = m_inPos.load(std::memory_order_relaxed);
		const unsigned next = (inPos + 1) % m_bufSize;
		if (next == m_outPos.load(std::memory_order_relaxed)) {
			return false;
		}

		std::atomic_thread_fence(std::memory_order_acquire);

		m_buffer[inPos] = val;

		std::atomic_thread_fence(std::memory_order_release);

		m_inPos.store(next, std::memory_order_relaxed);
		return true;
	}

	bool ICACHE_RAM_ATTR push(const T& val)
	{
		return push(T(val));
	}

	size_t push_n(const T* buffer, size_t size)
	{
		const auto inPos = m_inPos.load(std::memory_order_relaxed);
		const auto outPos = m_outPos.load(std::memory_order_relaxed);

		size_t blockSize = (outPos > inPos) ? outPos - 1 - inPos : (outPos == 0) ? m_bufSize - 1 - inPos : m_bufSize - inPos;
		blockSize = std::min(size, blockSize);
		if (!blockSize) return 0;
		int next = (inPos + blockSize) % m_bufSize;

		std::atomic_thread_fence(std::memory_order_acquire);

		auto dest = m_buffer.get() + inPos;
		std::copy_n(buffer, blockSize, dest);
		size = std::min(size - blockSize, outPos > 1 ? outPos - next - 1 : 0);
		next += size;
		dest = m_buffer.get();
		std::copy_n(buffer + blockSize, size, dest);

		std::atomic_thread_fence(std::memory_order_release);

		m_inPos.store(next, std::memory_order_relaxed);
		return blockSize + size;
	}

	T pop()
	{
		const auto outPos = m_outPos.load(std::memory_order_relaxed);
		if (m_inPos.load(std::memory_order_relaxed) == outPos) return defaultValue;

		std::atomic_thread_fence(std::memory_order_acquire);

		const auto val = m_buffer[outPos];

		std::atomic_thread_fence(std::memory_order_release);

		m_outPos.store((outPos + 1) % m_bufSize, std::memory_order_relaxed);
		return val;
	}

	size_t pop_n(T* buffer, size_t size) {
		size_t avail = size = std::min(size, available());
		if (!avail) return 0;
		const auto outPos = m_outPos.load(std::memory_order_relaxed);
		size_t n = std::min(avail, m_bufSize - outPos);

		std::atomic_thread_fence(std::memory_order_acquire);

		buffer = std::copy_n(m_buffer.get() + outPos, n, buffer);
		avail -= n;
		std::copy_n(m_buffer.get(), avail, buffer);

		std::atomic_thread_fence(std::memory_order_release);

		m_outPos.store((outPos + size) % m_bufSize, std::memory_order_relaxed);
		return size;
	}

protected:
	const T defaultValue = {};
	unsigned m_bufSize;
	std::unique_ptr<T[] > m_buffer;
	std::atomic<unsigned> m_inPos;
	std::atomic<unsigned> m_outPos;
};

template< typename T > class circular_queue_mp : protected circular_queue<T>
{
public:
	circular_queue_mp() = default;
	circular_queue_mp(const size_t capacity) : circular_queue<T>(capacity)
	{}
	circular_queue_mp(circular_queue<T>&& cq) : circular_queue<T>(std::move(cq))
	{}
	using circular_queue<T>::operator=;
	using circular_queue<T>::capacity;
	using circular_queue<T>::flush;
	using circular_queue<T>::available;
	using circular_queue<T>::available_for_push;
	using circular_queue<T>::peek;
	using circular_queue<T>::pop;
	using circular_queue<T>::pop_n;

	bool capacity(const size_t cap)
	{
#ifdef ESP8266
		InterruptLock lock;
#else
		std::lock_guard<std::mutex> lock(m_pushMtx);
#endif
		return circular_queue<T>::capacity(cap);
	}

	bool ICACHE_RAM_ATTR push(T&& val)
	{
#ifdef ESP8266
		InterruptLock lock;
#else
		std::lock_guard<std::mutex> lock(m_pushMtx);
#endif
		return circular_queue<T>::push(std::move(val));
	}

	bool ICACHE_RAM_ATTR push(const T& val)
	{
#ifdef ESP8266
		InterruptLock lock;
#else
		std::lock_guard<std::mutex> lock(m_pushMtx);
#endif
		return circular_queue<T>::push(val);
	}

	size_t push_n(const T* buffer, size_t size)
	{
#ifdef ESP8266
		InterruptLock lock;
#else
		std::lock_guard<std::mutex> lock(m_pushMtx);
#endif
		return circular_queue<T>::push_n(buffer, size);
	}

	T pop_revenant()
	{
#ifdef ESP8266
		InterruptLock lock;
#else
		std::lock_guard<std::mutex> lock(m_pushMtx);
#endif
		const auto outPos = circular_queue<T>::m_outPos.load(std::memory_order_relaxed);
		const auto inPos = circular_queue<T>::m_inPos.load(std::memory_order_relaxed);
		std::atomic_thread_fence(std::memory_order_acquire);
		if (inPos == outPos) return circular_queue<T>::defaultValue;
		const auto val = circular_queue<T>::m_buffer[inPos] = circular_queue<T>::m_buffer[outPos];
		const auto bufSize = circular_queue<T>::m_bufSize;
		std::atomic_thread_fence(std::memory_order_release);
		circular_queue<T>::m_outPos.store((outPos + 1) % bufSize, std::memory_order_relaxed);
		circular_queue<T>::m_inPos.store((inPos + 1) % bufSize, std::memory_order_relaxed);
		return val;
	}

#ifndef ESP8266
protected:
	std::mutex m_pushMtx;
#endif
};

#endif // __circular_queue_h
