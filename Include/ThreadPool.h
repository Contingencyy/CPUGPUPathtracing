#pragma once

#include <functional>

namespace ThreadPool
{

	struct JobDispatchArgs
	{
		uint32_t job_index;
		uint32_t group_index;
	};

	void Init();
	void Exit();

	void QueueJob(const std::function<void()>& func);
	void Dispatch(uint32_t num_jobs, uint32_t group_size, const std::function<void(JobDispatchArgs)>& job);

	bool IsBusy();
	void WaitAll();

}
