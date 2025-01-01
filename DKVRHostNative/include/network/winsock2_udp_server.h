#pragma once

#ifdef _WIN32
#	include <WinSock2.h>
#else
#	error "Do not include this header in non-Windows OS."
#endif

#include "network/udp_server.h"
#include "util/logger.h"
#include "util/thread_container.h"

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
		void SendAndRecvMessage();

		bool PeekRecv();
		void HandleRecv();
		bool PeekWritability();
		void SendOneDatagram();

		ThreadContainer<Winsock2UDPServer> winsock_thread_;

		WSADATA wsa_data_;
		SOCKET socket_;
		unsigned long binding_ip_;

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr