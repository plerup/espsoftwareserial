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

#include "task.h"
#include "task_completion_source.h"

#include <vector>

namespace ghostl
{
	template<typename T = void>
	struct when_all
	{
		std::vector<ghostl::task<>> continuations;
		std::vector<T> results;
		std::atomic<size_t> remainingCnt{ 0 };
		ghostl::task_completion_source<std::vector<T>> tcs;

		auto run_task(ghostl::task<T> t, size_t pos) -> ghostl::task<>
		{
			results[pos] = std::move(co_await t);
			if (--remainingCnt == 0)
			{
				tcs.set_value(std::move(results));
				if (auto handle = tcs.handle(); handle && !handle.done()) {
					handle.resume();
				}
			}
			co_return;
		}

		when_all() = delete;
		template<typename C> explicit when_all(C&& tasks)
		{
			for (auto& task : tasks)
			{
				continuations.emplace_back(run_task(std::exchange(task, {}), remainingCnt++));
			}
			results.resize(remainingCnt);
			for (auto& task : continuations)
			{
				task.resume();
			}
		}
		when_all(const when_all& other) = delete;
		when_all(when_all&& other) = delete;
		when_all& operator=(when_all& other) = delete;
		when_all& operator=(when_all&& other) = delete;
		auto operator ()() {
			return tcs.token();
		}
	};

	template<>
	struct when_all<>
	{
		std::vector<ghostl::task<>> continuations;
		std::atomic<size_t> remainingCnt{ 0 };
		ghostl::task_completion_source<> tcs;

		auto run_task(ghostl::task<> t) -> ghostl::task<>
		{
			co_await t;
			if (--remainingCnt == 0)
			{
				tcs.set_value();
				if (auto handle = tcs.handle(); handle && !handle.done()) {
					handle.resume();
				}
			}
			co_return;
		}

		when_all() = delete;
		template<typename C> explicit when_all(C&& tasks)
		{
			for (auto& task : tasks)
			{
				continuations.emplace_back(run_task(std::exchange(task, {})));
				++remainingCnt;
			}
			for (auto& task : continuations)
			{
				task.resume();
			}
		}
		when_all(const when_all& other) = delete;
		when_all(when_all&& other) = delete;
		when_all& operator=(when_all& other) = delete;
		when_all& operator=(when_all&& other) = delete;
		auto operator ()() {
			return tcs.token();
		}
	};
} // namespace ghostl
