#pragma once
/*
generator.h - Implementation of a C++20 generator.
Copyright (c) 2023 Dirk O. Kaar. All rights reserved.

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

#include <atomic>
#include <memory>
#include <coroutine>

namespace ghostl
{
    template <typename T>
    struct generator
    {
        struct promise_type
        {
            generator get_return_object() noexcept
            {
                return generator(std::coroutine_handle<promise_type>::from_promise(*this));
            }
            std::suspend_always initial_suspend() const { return {}; }
            std::suspend_always final_suspend() const noexcept { return {}; }
            void unhandled_exception() { exception = std::current_exception(); }

            template<typename C>
            std::suspend_always yield_value(C&& from)
            {
                value = std::move(from); // caching the result in promise
                return {};
            }
            void return_void() const { }

            T value{};
            std::exception_ptr exception;
        };

        std::coroutine_handle<promise_type> coroutine;

        generator() noexcept : coroutine(nullptr) {}
        explicit generator(std::coroutine_handle<promise_type> h) : coroutine(h) { }
        generator(const generator&) = delete;
        generator(generator&& other) noexcept : coroutine(std::exchange(other.coroutine, nullptr)), full(other.full) {};
        ~generator() { if (coroutine) coroutine.destroy(); }
        generator& operator=(const generator&) = delete;
        generator& operator=(generator&& other) noexcept
        {
            if (std::addressof(other) != this)
            {
                if (coroutine) coroutine.destroy();
                coroutine = std::exchange(other.coroutine, nullptr);
                full = other.full;
            }
            return *this;
        }
        explicit operator bool()
        {
            fill();
            return !coroutine.done();
        }
        T operator()()
        {
            fill();
            full = false;
            return std::move(coroutine.promise().value);
        }
    private:
        bool full = false;

        void fill()
        {
            if (!full)
            {
                coroutine();
                // propagate coroutine exception in called context
                if (coroutine.promise().exception)
                    std::rethrow_exception(coroutine.promise().exception);
                full = true;
            }
        }
    };
} // namespace ghostl
