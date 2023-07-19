#pragma once
/*
run_task.h - Implementation of a C++20 async coroutines task runner featuring continue_with.
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

#include <functional>
#include <atomic>

namespace ghostl
{
	template<typename T = void> struct run_task {
		using task_type = ghostl::task<T>;
		using continuation_type = std::function<void(T)>;
		std::shared_ptr<ghostl::task<>> task = std::make_shared<ghostl::task<>>();
		std::shared_ptr<continuation_type> continuation = std::make_shared<continuation_type>();
		static auto coroutine(decltype(task) t, decltype(continuation) cont, task_type to_run) -> ghostl::task<> {
			auto res = co_await std::move(to_run);
			if (*cont) (*cont)(std::move(res));
			t.reset();
		}
		run_task() = delete;
		explicit run_task(task_type&& to_run) { *task = coroutine(task, continuation, std::move(to_run)); }

		run_task(const run_task&) = delete;
		run_task(run_task&& other) noexcept = delete;
		auto operator=(const run_task&)->run_task & = delete;
		auto operator=(run_task&& other) noexcept -> run_task & = delete;
		~run_task() {}

		run_task& continue_with(continuation_type cont) { *continuation = cont; return *this; }
		void resume() { task->resume(); }
	};
	template<> struct run_task<void> {
		using task_type = ghostl::task<>;
		using continuation_type = std::function<void()>;
		std::shared_ptr<ghostl::task<>> task = std::make_shared<ghostl::task<>>();
		std::shared_ptr<continuation_type> continuation = std::make_shared<continuation_type>();
		static auto coroutine(decltype(task) t, decltype(continuation) cont, task_type to_run) -> ghostl::task<> {
			co_await std::move(to_run);
			if (*cont) (*cont)();
			t.reset();
		}
		run_task() = delete;
		explicit run_task(task_type&& to_run) { *task = coroutine(task, continuation, std::move(to_run)); }

		run_task(const run_task&) = delete;
		run_task(run_task&& other) noexcept = delete;
		auto operator=(const run_task&)->run_task & = delete;
		auto operator=(run_task&& other) noexcept -> run_task & = delete;
		~run_task() {}

		run_task& continue_with(continuation_type cont) { *continuation = cont; return *this; }
		void resume() { task->resume(); }
	};
} // namespace ghostl
