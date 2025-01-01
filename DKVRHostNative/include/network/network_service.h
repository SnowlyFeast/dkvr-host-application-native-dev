#pragma once

#include <memory>

#include "network/datagram.h"
#include "network/udp_server.h"
#include "util/logger.h"
#include "util/thread_container.h"

namespace dkvr 
{

	class NetworkService final
	{
	public:
		NetworkService();
		~NetworkService();

		bool Run(unsigned long ip = 0, unsigned short port = 8899u);
		void Stop();
		bool WaitAndPopReceived(unsigned long& address_out, Instruction& inst_out);
		void Send(unsigned long address, Instruction& inst);
		void RequestWakeup() { udp_->Wakeup(); }

	private:
		NetworkService(const NetworkService&) = delete;
		NetworkService(NetworkService&&) = delete;
		void operator= (const NetworkService&) = delete;
		void operator= (NetworkService&&) = delete;

		void CheckAndRepairService();

		std::unique_ptr<UDPServer> udp_;
		ThreadContainer<NetworkService> watchdog_thread_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr