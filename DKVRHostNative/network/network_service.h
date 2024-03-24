#pragma once

#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>

#include "datagram.h"
#include "udp_server.h"
#include "util/logger.h"

namespace dkvr {

	class NetworkService
	{
	public:
		static NetworkService& GetInstance();

		bool Init();
		bool Run(unsigned short port = 8899);
		unsigned long WaitAndPopReceived(Instruction& out);
		void QueueSending(unsigned long address, Instruction& inst);
		void RequestWakeup() { udp_->Wakeup(); }

	private:
		NetworkService();
		NetworkService(const NetworkService&) = delete;
		NetworkService(NetworkService&&) = delete;
		~NetworkService();
		void operator= (const NetworkService&) = delete;
		void operator= (NetworkService&&) = delete;

		UDPServer* udp_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr