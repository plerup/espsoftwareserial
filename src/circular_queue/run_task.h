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
	namespace details
	{
		struct then_task final
		{
			struct promise_type final
			{
				constexpr then_task get_return_object() const noexcept { return {}; }
				constexpr std::suspend_never initial_suspend() const noexcept { return {}; }
				void unhandled_exception() const { std::rethrow_exception(std::current_exception()); }
				constexpr void return_void() const noexcept {}
				constexpr std::suspend_never final_suspend() const noexcept { return {}; }
			};
		};
	}
	template<typename T = void>
	struct run_task
	{
		using task_type = ghostl::task<T>;
		using continuation_type = std::function<void(T)>;

		static auto coroutine(std::shared_ptr<task_type> task_ptr, std::shared_ptr<continuation_type> continuation) -> ghostl::details::then_task
		{
			auto res = co_await *task_ptr;
			if (continuation && *continuation) (*continuation)(std::move(res));
		}

		std::shared_ptr<task_type> to_run_ptr;
		std::shared_ptr<continuation_type> continuation;

		run_task() = delete;
		explicit run_task(task_type&& to_run)
		{
			to_run_ptr = std::make_shared<task_type>(std::move(to_run));
		}

		run_task(const run_task&) = delete;
		run_task(run_task&& other) noexcept = default;
		auto operator=(const run_task&)->run_task & = delete;
		auto operator=(run_task&& other) noexcept -> run_task & = default;
		~run_task() {}

		run_task& continue_with(continuation_type cont)
		{
			continuation = std::make_shared<continuation_type>(cont);
			return *this;
		}
		void resume()
		{
			coroutine(to_run_ptr, continuation);
		}
	};
	template<>
	struct run_task<void>
	{
		using task_type = ghostl::task<>;
		using continuation_type = std::function<void()>;

		static auto coroutine(std::shared_ptr<task_type> task_ptr, std::shared_ptr<continuation_type> continuation) -> ghostl::details::then_task
		{
			co_await *task_ptr;
			if (continuation && *continuation) (*continuation)();
		}

		std::shared_ptr<task_type> to_run_ptr;
		std::shared_ptr<continuation_type> continuation;

		run_task() = delete;
		explicit run_task(task_type&& to_run)
		{
			to_run_ptr = std::make_shared<task_type>(std::move(to_run));
		}

		run_task(const run_task&) = delete;
		run_task(run_task&& other) noexcept = default;
		auto operator=(const run_task&)->run_task & = delete;
		auto operator=(run_task&& other) noexcept -> run_task & = default;
		~run_task() {}

		run_task& continue_with(continuation_type cont)
		{
			continuation = std::make_shared<continuation_type>(cont);
			return *this;
		}
		void resume()
		{
			coroutine(to_run_ptr, continuation);
		}
	};
} // namespace ghostl
