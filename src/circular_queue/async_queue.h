#pragma once
/*
async_queue.h - Implementation of a C++20 async awaitable queue.
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

#include "circular_queue/task_completion_source.h"
#include "circular_queue/task.h"
#include "circular_queue/lfllist.h"

#if defined(__GNUC__)
#undef ALWAYS_INLINE_ATTR
#define ALWAYS_INLINE_ATTR __attribute__((always_inline))
#else
#define ALWAYS_INLINE_ATTR
#endif

namespace ghostl
{
    template<typename T = void>
    struct async_queue : private lfllist<task_completion_source<T>>
    {
        async_queue()
        {
            task_completion_source<T> tcs;
            pending_tcs = lfllist<task_completion_source<T>>::emplace_front(std::move(tcs));
        }
        async_queue(const async_queue&) = delete;
        async_queue(async_queue&&) = delete;
        ~async_queue() = default;
        auto operator =(const async_queue&)->async_queue & = delete;
        auto operator =(async_queue&&)->async_queue & = delete;

        [[nodiscard]] auto push(T&& val) -> bool
        {
            decltype(pending_tcs) next_tcs;
            if (!(next_tcs = lfllist<task_completion_source<T>>::emplace_front(task_completion_source<T>()))) return false;
            pending_tcs.exchange(next_tcs)->item.set_value(std::move(val));
            return true;
        }
        inline auto push(const T& val) -> bool ALWAYS_INLINE_ATTR
        {
            T v(val);
            return push(std::move(v));
        }
        auto flush() -> void
        {
            bool is_back = true;
            for_each([&is_back](lfllist<task_completion_source<T>>::node_type* const to_erase) { if (std::exchange(is_back, false)) erase(to_erase); });
        }
        auto pop() -> ghostl::task<T>
        {
            auto next = lfllist<task_completion_source<T>>::back();
            auto val = co_await next->item.token();
            lfllist<task_completion_source<T>>::erase(next);
            co_return val;
        }

    private:
        std::atomic<typename lfllist<task_completion_source<T>>::node_type*> pending_tcs;
    };
}
