#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <thread>
#ifdef _WIN32
#	include <WinSock2.h>
#else
#	error "Do not include this header in non-Windows OS."
#endif

#include "network/udp_server.h"
#include "util/logger.h"

namespace dkvr {

	class Winsock2UDPServer final : public UDPServer
	{
	public:
		Winsock2UDPServer();
		~Winsock2UDPServer();

	protected:
		int InternalInit() override;
		int InternalBind() override;
		void InternalClose() override;
		void InternalDeinit() override;

	private:
		void ParseWSAError(int wsa_error);
		void NetworkThreadLoop();

		bool PeekRecv();
		void HandleRecv();
		bool PeekWritability();
		void SendOneDatagram();

		WSADATA wsa_data_;
		SOCKET socket_;
		std::unique_ptr<std::thread> net_thread_;
		std::atomic_bool exit_flag_;
		std::map<unsigned long, int> arrivals_;
		unsigned long binding_ip_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr