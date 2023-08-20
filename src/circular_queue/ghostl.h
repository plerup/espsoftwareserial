#pragma once
/*
ghostl.h - Implementation of a bare-bones, mostly no-op, C++ STL shell
           that allows building some Arduino ESP8266/ESP32
           libraries on Aruduino AVR.
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

#ifndef __ghostl_h
#define __ghostl_h

#if defined(ARDUINO_ARCH_SAMD)
#include <atomic>
#endif

using size_t = decltype(sizeof(char));

namespace std
{
#if !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266)
    typedef enum memory_order {
        memory_order_relaxed,
        memory_order_acquire,
        memory_order_release,
        memory_order_seq_cst
    } memory_order;

    template< typename T > class atomic {
    private:
        T value;
    public:
        atomic() {}
        atomic(T desired) { value = desired; }

        void store(T desired, std::memory_order = std::memory_order_seq_cst) volatile noexcept { value = desired; }

        T load(std::memory_order = std::memory_order_seq_cst) const volatile noexcept { return value; }

        T exchange(T desired, std::memory_order = std::memory_order_seq_cst) const volatile noexcept {
            noInterrupts();
            T orig = value;
            value = desired;
            interrupts();
            return orig;
        }

        bool compare_exchange_strong(T& expected, T desired, std::memory_order order = std::memory_order_seq_cst) volatile noexcept {
            noInterrupts();
            const bool equal = value == expected;
            if (equal) value = desired;
            else expected = value;
            interrupts();
            return equal;
        }

        bool compare_exchange_weak(T& expected, T desired, std::memory_order order = std::memory_order_seq_cst) volatile noexcept {
            return compare_exchange_strong(expected, desired, order);
        };
    };

    inline void atomic_thread_fence(std::memory_order order) noexcept {}

    template< typename T > T&& move(T& t) noexcept { return static_cast<T&&>(t); }
#endif

#ifndef ESP8266
    template< typename T, size_t long N > struct array
    {
        T _M_elems[N];
        decltype(sizeof(0)) size() const { return N; }
        T& operator[](decltype(sizeof(0)) i) { return _M_elems[i]; }
        const T& operator[](decltype(sizeof(0)) i) const { return _M_elems[i]; }
    };

    template< typename T > class unique_ptr
    {
    public:
        using pointer = T*;
        unique_ptr() noexcept : ptr(nullptr) {}
        unique_ptr(pointer p) : ptr(p) {}
        pointer operator->() const noexcept { return ptr; }
        T& operator[](decltype(sizeof(0)) i) const { return ptr[i]; }
        void reset(pointer p = pointer()) noexcept
        {
            delete ptr;
            ptr = p;
        }
        T& operator*() const { return *ptr; }
    private:
        pointer ptr;
    };

    template< typename T > using function = T*;
    using nullptr_t = decltype(nullptr);

    template<typename T>
    struct identity {
        typedef T type;
    };

    template <typename T>
    inline T&& forward(typename identity<T>::type& t) noexcept
    {
        return static_cast<typename identity<T>::type&&>(t);
    }
#endif // ESP8266

#ifdef ESP8266
#if defined (__cplusplus)
    extern "C" {
#endif
        bool __atomic_compare_exchange_4(uint32_t* ptr, uint32_t* expected, uint32_t desired,
            bool weak, int success_memorder, int failure_memorder)
        {
            (void)weak;
            (void)success_memorder;
            (void)failure_memorder;
            noInterrupts();
            const bool equal = *ptr == *expected;
            if (equal) *ptr = desired;
            else *expected = *ptr;
            interrupts();
            return equal;
        }

        bool __atomic_compare_exchange_1(uint8_t* ptr, uint8_t* expected, uint8_t desired,
            bool weak, int success_memorder, int failure_memorder)
        {
            (void)weak;
            (void)success_memorder;
            (void)failure_memorder;
            noInterrupts();
            const bool equal = *ptr == *expected;
            if (equal) *ptr = desired;
            else *expected = *ptr;
            interrupts();
            return equal;
        }

        uint32_t __atomic_exchange_4(uint32_t* ptr, uint32_t value, int memorder)
        {
            (void)memorder;
            noInterrupts();
            uint32_t orig = *ptr;
            *ptr = value;
            interrupts();
            return orig;
        }

        uint8_t __atomic_exchange_1(uint8_t* ptr, uint8_t value, int memorder)
        {
            (void)memorder;
            noInterrupts();
            uint8_t orig = *ptr;
            *ptr = value;
            interrupts();
            return orig;
        }

        uint32_t __atomic_fetch_add_4(uint32_t* ptr, uint32_t value, int memorder)
        {
            (void)memorder;
            noInterrupts();
            uint32_t orig = *ptr;
            *ptr += value;
            interrupts();
            return orig;
        }

        uint32_t __atomic_fetch_sub_4(uint32_t* ptr, uint32_t value, int memorder)
        {
            (void)memorder;
            noInterrupts();
            uint32_t orig = *ptr;
            *ptr -= value;
            interrupts();
            return orig;
        }
#if defined (__cplusplus)
    } // extern "C"
#endif
#endif // ESP8266
}
#endif // __ghostl_h
