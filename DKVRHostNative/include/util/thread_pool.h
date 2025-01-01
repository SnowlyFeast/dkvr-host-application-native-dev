#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace dkvr {

	class ThreadPool
	{
	public:
		static ThreadPool& GetInstance();

		void Terminate();
		void Queue(std::function<void()>&);

		bool busy();


	private:
		ThreadPool(size_t size = 4);
		ThreadPool(const ThreadPool&) = delete;
		ThreadPool(ThreadPool&&) = delete;
		virtual ~ThreadPool();
		void operator= (const ThreadPool&) = delete;
		void operator= (ThreadPool&&) = delete;

		void ThreadLoop();

		bool terminated_;
		std::mutex mutex_;
		std::condition_variable convar_;
		std::vector<std::thread> threads_;
		std::queue<std::function<void()>> tasks_;
	};

}	// namespace dkvr