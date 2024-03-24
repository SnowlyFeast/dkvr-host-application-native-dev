#include "thread_pool.h"

#include <functional>
#include <mutex>
#include <stdexcept>

namespace dkvr {
	namespace util {

		ThreadPool& ThreadPool::GetInstance()
		{
			static ThreadPool instance(4);
			return instance;
		}

		ThreadPool::ThreadPool(size_t size) : terminated_(false), mutex_(), convar_(), threads_(), tasks_()
		{
			threads_.reserve(size);
			for (size_t i = 0; i < size; i++)
				threads_.emplace_back(std::thread(&ThreadPool::ThreadLoop, this));
		}

		ThreadPool::~ThreadPool()
		{
			Terminate();
		}

		void ThreadPool::Terminate()
		{
			{
				std::unique_lock<std::mutex> lock(mutex_);
				terminated_ = true;
			}
			convar_.notify_all();
			for (std::thread& t : threads_)
				t.join();
			threads_.clear();
		}

		void ThreadPool::Queue(std::function<void()>& task)
		{
			{
				std::unique_lock<std::mutex> lock(mutex_);
				if (terminated_)
					throw std::runtime_error("thread pool terminated.");
				tasks_.push(task);
			}
			convar_.notify_one();
		}

		bool ThreadPool::busy()
		{
			std::unique_lock<std::mutex> lock(mutex_);
			return !tasks_.empty();
		}

		void ThreadPool::ThreadLoop()
		{
			while (true) {
				std::unique_lock<std::mutex> lock(mutex_);
				convar_.wait(lock, [&] {return !tasks_.empty() || terminated_; });
				if (terminated_)
					return;
				std::function<void()> task(tasks_.front());
				tasks_.pop();
				lock.unlock();
				task();
			}
		}

	}	// namespace util
}	// namespace dkvr