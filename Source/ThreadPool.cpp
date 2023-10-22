#include "Common.h"
#include "ThreadPool.h"

#include <thread>
#include <condition_variable>

namespace ThreadPool
{

	template<typename T>
	class ThreadSafeRingBuffer
	{
	public:
		static constexpr uint32_t THREAD_SAFE_RING_BUFFER_CAPACITY = 512;

	public:
		bool PushBack(const T& job)
		{
			bool result = false;
			lock.lock();
			size_t next = (head + 1) % THREAD_SAFE_RING_BUFFER_CAPACITY;

			if (next != tail)
			{
				jobs[head] = job;
				head = next;
				result = true;
			}

			lock.unlock();
			return result;
		}

		bool PopFront(T& job)
		{
			bool result = false;
			lock.lock();

			if (tail != head)
			{
				job = jobs[tail];
				tail = (tail + 1) % THREAD_SAFE_RING_BUFFER_CAPACITY;
				result = true;
			}

			lock.unlock();
			return result;
		}

	private:
		std::array<T, THREAD_SAFE_RING_BUFFER_CAPACITY> jobs;
		size_t head = 0;
		size_t tail = 0;
		std::mutex lock;

	};

	struct Data
	{
		std::vector<std::thread> threads;
		std::condition_variable wake_cond;
		std::mutex wake_mutex;
		uint64_t jobs_enqueued = 0;
		std::atomic<uint64_t> jobs_completed;

		bool exit = false;

		ThreadSafeRingBuffer<std::function<void()>> job_buffer;
	} static data;

	static void Poll()
	{
		data.wake_cond.notify_one();
		std::this_thread::yield();
	}

	void Init()
	{
		data.jobs_completed.store(0);

		uint32_t num_cores = std::thread::hardware_concurrency();
		uint32_t num_threads = std::max(1u, num_cores);

		data.threads.reserve(num_threads);

		for (size_t i = 0; i < num_threads; ++i)
		{
			data.threads.push_back(std::thread([] {
				std::function<void()> job;

				while (!data.exit)
				{
					if (data.job_buffer.PopFront(job))
					{
						job();
						data.jobs_completed.fetch_add(1);
					}
					else
					{
						std::unique_lock<std::mutex> lock(data.wake_mutex);
						data.wake_cond.wait(lock);
					}
				}
			}));

			data.threads[i].detach();
		}
	}

	void Exit()
	{
		data.exit = false;
		
		for (auto& thread : data.threads)
		{
			if (thread.joinable())
			{
				thread.join();
			}
		}
	}

	void QueueJob(const std::function<void()>& job)
	{
		data.jobs_enqueued++;

		while (!data.job_buffer.PushBack(job))
		{
			Poll();
		}

		data.wake_cond.notify_one();
	}

	void Dispatch(uint32_t num_jobs, uint32_t group_size, const std::function<void(JobDispatchArgs)>& job)
	{
		if (num_jobs == 0 || group_size == 0)
			return;

		const uint32_t group_count = (num_jobs + group_size - 1) / group_size;
		data.jobs_enqueued += group_count;

		for (size_t group_index = 0; group_index < group_count; ++group_index)
		{
			const auto& job_group = [num_jobs, group_size, job, group_index]() {
				const uint32_t job_group_offset = group_index * group_size;
				const uint32_t job_group_end = std::min(job_group_offset + group_size, num_jobs);

				JobDispatchArgs dispatch_args = {};
				dispatch_args.group_index = group_index;

				for (size_t i = job_group_offset; i < job_group_end; ++i)
				{
					dispatch_args.job_index = i;
					job(dispatch_args);
				}
			};

			while (!data.job_buffer.PushBack(job_group))
			{
				Poll();
			}

			data.wake_cond.notify_one();
		}
	}

	bool IsBusy()
	{
		return data.jobs_completed.load() < data.jobs_enqueued;
	}

	void WaitAll()
	{
		while (IsBusy())
		{
			Poll();
		}
	}

}
