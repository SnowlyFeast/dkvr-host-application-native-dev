#pragma once

#include <atomic>
#include <memory>
#include <thread>

namespace dkvr
{

    /**
     * @brief   Controller class for infinite looping thread.
     *          @c Run() will begin the thread that keep calling @c callback_.
     */
    template<typename T>
    class ThreadContainer
    {
    public:
        ThreadContainer(const T& caller) : caller_(caller) { }
        virtual ~ThreadContainer() { Stop(); }

        void Run()
        {
            if (thread_running_) return;
            if (callback_ == nullptr) return;

            thread_running_ = true;
            exit_flag_ = false;
            thread_ptr_ = std::make_unique<std::thread>(&ThreadContainer::ThreadLoop, this);
        }

        void Stop()
        {
            if (!thread_running_) return;

            if (thread_ptr_)
            {
                exit_flag_ = true;
                if (thread_ptr_->joinable())
                    thread_ptr_->join();
            }
            thread_ptr_.reset();
            thread_running_ = false;
        }

        void StopAsync()
        {
            if (!thread_running_) return;

            if (thread_ptr_)
            {
                exit_flag_ = true;
                if (thread_ptr_->joinable())
                    thread_ptr_->detach();
            }
            thread_ptr_.reset();
        }

        bool IsRunning() const { return thread_running_; }

        void operator+=(void(T::* callback)()) { callback_ = callback; }

    private:
        ThreadContainer(const ThreadContainer&) = delete;
        ThreadContainer(ThreadContainer&&) = delete;
        void operator=(const ThreadContainer&) = delete;
        void operator=(ThreadContainer&&) = delete;

        void ThreadLoop()
        {
            while (!exit_flag_)
                (const_cast<T&>(caller_).*callback_)();

            thread_running_ = false;    // for async stop
        }

        std::atomic_bool thread_running_ = false;
        std::atomic_bool exit_flag_ = false;
        std::unique_ptr<std::thread> thread_ptr_ = nullptr;

        void (T::* callback_)() = nullptr;
        const T& caller_;
    };

}   // namespace dkvr
