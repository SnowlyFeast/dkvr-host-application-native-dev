#pragma once

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <queue>

#include "network/datagram.h"
#include "network/udp_server.h"
#include "util/logger.h"

namespace dkvr 
{

	class NetworkService final
	{
	public:
		NetworkService();
		~NetworkService();

		bool Run(unsigned short port = 8899u);
		void Stop();
		unsigned long WaitAndPopReceived(Instruction& out);
		void Send(unsigned long address, Instruction& inst);
		void RequestWakeup() { udp_->Wakeup(); }

	private:
		NetworkService(const NetworkService&) = delete;
		NetworkService(NetworkService&&) = delete;
		void operator= (const NetworkService&) = delete;
		void operator= (NetworkService&&) = delete;

		std::unique_ptr<UDPServer> udp_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr