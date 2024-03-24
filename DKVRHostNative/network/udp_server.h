#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>

#include "datagram.h"
#include "util/logger.h"

namespace dkvr {

	class UDPServer
	{
	public:
		enum class Status
		{
			InitRequired,
			InitFailed,
			StandBy,
			Running,
			Closed,
			Error
		};

		UDPServer() : status_(Status::InitRequired), received_(), sending_(), exit_flag_(false), mutex_(), convar_(), port_(8899) { }
		virtual ~UDPServer() { }

		int Init();
		int Bind();
		void Close();
		int PushSending(const Datagram& dgram);
		bool PeekReceived() const;
		bool WaitReceived() const;
		Datagram PopReceived();
		void Wakeup() { convar_.notify_all(); }

		Status status() const { return status_; }
		unsigned short client_port() const { return 8899u; }
		unsigned short port() const { return port_; }
		void set_port(unsigned short port) { port_ = port; }

	protected:
		virtual int InternalInit() = 0;
		// responsible for handling recv and send
		virtual int InternalBind() = 0;
		virtual void InternalClose() = 0;

		bool PeekSending() const;
		Datagram PopSending();
		void PushReceived(const Datagram& dgram);
		void RaiseErrorStatus() { status_ = Status::Error; }

	private:
		UDPServer(const UDPServer&) = delete;
		UDPServer(UDPServer&&) = delete;
		void operator= (const UDPServer&) = delete;
		void operator= (UDPServer&&) = delete;

		Status status_;
		std::queue<Datagram> received_;
		std::queue<Datagram> sending_;
		std::atomic_bool exit_flag_;
		mutable std::mutex mutex_;
		mutable std::condition_variable convar_;
		unsigned short port_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr