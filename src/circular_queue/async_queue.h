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

namespace ghostl
{
    template<typename T = void>
    struct async_queue : private lfllist<T>
    {
        async_queue()
        {
            cur_tcs.store(tcs_queue.emplace_front(task_completion_source<>()));
        }
        async_queue(const async_queue&) = delete;
        async_queue(async_queue&&) = delete;
        auto operator =(const async_queue&)->async_queue & = delete;
        auto operator =(async_queue&&)->async_queue & = delete;

        [[nodiscard]] auto push(T&& val) -> bool
        {
            if (lfllist<T>::emplace_front(std::move(val)) == nullptr) return false;
            auto _cur_tcs = cur_tcs.exchange(tcs_queue.emplace_front(task_completion_source<>()));
            task_completion_source<> tcs = _cur_tcs->item;
            tcs.set_value();
            return true;
        }
        inline auto push(const T& val) -> bool ALWAYS_INLINE_ATTR
        {
            T v(val);
            return push(std::move(v));
        }
        auto flush() -> void
        {
            while (tcs_queue.back())
            {
                if (task_completion_source<> item; tcs_queue.try_pop(item)) {}
            }
            cur_tcs.store(tcs_queue.emplace_front(task_completion_source<>()));
            while (lfllist<T>::back())
            {
                if (T item; lfllist<T>::try_pop(item)) {}
            }
        }
        auto pop() -> ghostl::task<T>
        {
            auto tcs = tcs_queue.back();
            auto token = tcs->item.token();
            co_await token;
            tcs_queue.erase(tcs);
            decltype(lfllist<T>::back()) node = lfllist<T>::back();
            T item = std::move(node->item);
            lfllist<T>::erase(node);
            co_return item;
        }

    private:
        lfllist<task_completion_source<>> tcs_queue;
        std::atomic<lfllist<task_completion_source<>>::node_type*> cur_tcs;
    };
}
