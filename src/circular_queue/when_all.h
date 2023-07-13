#pragma once

#include <atomic>
#include <memory>
#include "task.h"
#include "task_completion_source.h"

namespace ghostl
{
	template<typename T>
	struct when_all
	{
		std::vector<ghostl::task<void>> continuations;
		std::atomic<size_t> completedCnt;
		ghostl::task_completion_source<std::string> tcs;

		static auto run_task(
			T& t,
			std::atomic<size_t>& completedCnt,
			ghostl::task_completion_source<std::string> tcs) -> ghostl::task<void>
		{
			co_await t;
			if (--completedCnt == 0)
			{
				Serial.println("when_all: completed");
				tcs.set_value("hello");
				if (auto handle = tcs.handle(); handle && !handle.done()) {
					Serial.println("handle.resume");
					handle.resume();
					Serial.println("handle.resumed");
				}
			}
		}

		when_all() = delete;
		explicit when_all(std::vector<T>& tasks) : completedCnt(tasks.size())
		{
			for (size_t i = 0; i < tasks.size(); ++i)
			{
				continuations.emplace_back(run_task(tasks[i], completedCnt, tcs));
				continuations[i].resume();
			}
		}
		when_all(const when_all& other) = default;
		when_all(when_all&& other) = default;
		when_all& operator=(when_all& other) = default;
		when_all& operator=(when_all&& other) = default;
		auto token() {
			return tcs.token();
		}
	};
} // namespace ghostl
