#pragma once
/*
when_all.h - Implementation of a C++20 async coroutines when-all awaiter.
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

#include "run_task.h"
#include "task_completion_source.h"

#include <vector>

namespace ghostl
{
    template<typename T = void>
    struct when_all final
    {
        static auto continuation(T result, size_t pos,
            std::shared_ptr<std::vector<T>> results,
            std::shared_ptr<std::atomic<size_t>> remaining,
            ghostl::task_completion_source<std::vector<T>> tcs) -> void
        {
            (*results)[pos] = std::move(result);
            if (auto isremaining = -- * remaining; !isremaining)
            {
                tcs.set_value(std::move(*results));
            }
        }

        when_all() = delete;
        template<typename C> explicit when_all(C&& _tasks)
        {
            C tasks = std::move(_tasks);
            auto results = this->results;
            auto remaining = this->remaining;
            auto tcs = this->tcs;
            for (ghostl::task<T>& task : tasks)
            {
                auto pos = remaining->load();
                ++*remaining;
                auto runner = ghostl::run_task<T>(std::move(std::exchange(task, {})));
                runner.continue_with([pos, results, remaining, tcs](T result) { continuation(result, pos, results, remaining, tcs); });
                continuations->emplace_back(std::move(runner));
            }
            results->resize(remaining->load());
            if (continuations->empty()) tcs.set_value(std::move(*results));
            else for (auto& runner : *continuations)
            {
                runner.resume();
            }
        }
        template<typename C> explicit when_all(const C& tasks) = delete;
        when_all(const when_all& other) = delete;
        when_all(when_all&& other) = delete;
        when_all& operator=(when_all& other) = delete;
        when_all& operator=(when_all&& other) = delete;
        auto operator ()() {
            return tcs.token();
        }
    private:
        std::shared_ptr<std::vector<ghostl::run_task<T>>> continuations{
            std::make_shared<std::vector<ghostl::run_task<T>>>() };
        std::shared_ptr<std::vector<T>> results = std::make_shared<std::vector<T>>();
        std::shared_ptr<std::atomic<size_t>> remaining{
            std::make_shared<std::atomic<size_t>>(0) };
        ghostl::task_completion_source<std::vector<T>> tcs{};
    };
    template<>
    struct when_all<void> final
    {
        static auto continuation(
            std::shared_ptr<std::atomic<size_t>> remaining,
            ghostl::task_completion_source<> tcs) -> void
        {
            if (auto isremaining = -- * remaining; !isremaining)
            {
                tcs.set_value();
            }
        }

        when_all() = delete;
        template<typename C> explicit when_all(C&& _tasks)
        {
            C tasks = std::move(_tasks);
            auto remaining = this->remaining;
            auto tcs = this->tcs;
            for (ghostl::task<>& task : tasks)
            {
                ++*remaining;
                auto runner = ghostl::run_task<>(std::move(std::exchange(task, {})));
                runner.continue_with([remaining, tcs]() { continuation(remaining, tcs); });
                continuations->emplace_back(std::move(runner));
            }
            if (continuations->empty()) tcs.set_value();
            else for (auto& runner : *continuations)
            {
                runner.resume();
            }
        }
        template<typename C> explicit when_all(const C& tasks) = delete;
        when_all(const when_all& other) = delete;
        when_all(when_all&& other) = delete;
        when_all& operator=(when_all& other) = delete;
        when_all& operator=(when_all&& other) = delete;
        auto operator ()() {
            return tcs.token();
        }
    private:
        std::shared_ptr<std::vector<ghostl::run_task<>>> continuations{
            std::make_shared<std::vector<ghostl::run_task<>>>() };
        std::shared_ptr<std::atomic<size_t>> remaining{
            std::make_shared<std::atomic<size_t>>(0) };
        ghostl::task_completion_source<> tcs{};
    };
} // namespace ghostl
