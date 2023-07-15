#pragma once

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
