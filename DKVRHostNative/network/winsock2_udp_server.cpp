#include "winsock2_udp_server.h"

#include <chrono>
#include <sstream>
#include <thread>
#ifdef _WIN32
#	include <WinSock2.h>
#	include <WS2tcpip.h>
#	pragma comment(lib, "Ws2_32.lib")
#else
#	error "Do not include this header in non-Windows OS."
#endif

#include "net_result.h"

namespace dkvr {

	static void IncreaseDelay(std::chrono::milliseconds&);
	static void DecreaseDelay(std::chrono::milliseconds&);
	constexpr long long kThreadDelayUpperLimit = 512;
#ifdef DKVR_SYSTEM_ENABLE_FULL_THROTTLE
	constexpr long long kThreadDelayLowerLimit = 8;
#else
	constexpr long long kThreadDelayLowerLimit = 16;
#endif
	constexpr long long kYieldDurationLimit = 10;


	Winsock2UDPServer::Winsock2UDPServer() : wsa_data_{}, socket_(INVALID_SOCKET), net_thread_(nullptr), exit_flag_(false), arrivals_() { }

	Winsock2UDPServer::~Winsock2UDPServer()
	{
		InternalClose();
	}

	static unsigned long testip = 0;

	int Winsock2UDPServer::InternalInit()
	{
		// WSA start up
		int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
		if (wsa_result != 0) {
			// WSAStartup failed
			WSACleanup();
			logger_.Error("WSA startup failed : {}", wsa_result);
			ParseWSAError(wsa_result);

			return NetResult::InitFailed;
		}

		// get local ip address
		addrinfo hints{
			.ai_flags = AI_PASSIVE,
			.ai_family = AF_INET,
			.ai_socktype = SOCK_DGRAM,
			.ai_protocol = IPPROTO_UDP,
		};
		char hostname[256];
		if (!gethostname(hostname, sizeof(hostname))) {
			addrinfo* result;
			if (!getaddrinfo(hostname, std::to_string(port()).c_str(), &hints, &result)) {
				for (addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
					if (ptr->ai_family == AF_INET) {
						unsigned long ip = reinterpret_cast<sockaddr_in*>(ptr->ai_addr)->sin_addr.s_addr;
						unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
						logger_.Info("Host ip address is {:d}.{:d}.{:d}.{:d}", ptr[0], ptr[1], ptr[2], ptr[3]);
						testip = ip;
					}
				}
			}
			freeaddrinfo(result);
		}

		// create socket
		socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (socket_ == INVALID_SOCKET) {
			// socket failed-
			int error = WSAGetLastError();
			WSACleanup();
			logger_.Error("Socket creation failed : {}", error);
			ParseWSAError(error);

			return NetResult::InitFailed;
		}

		return NetResult::OK;
	}

	int Winsock2UDPServer::InternalBind()
	{
		sockaddr_in server_addr{
			.sin_family = AF_INET,
			.sin_port = htons(port())
		};
		inet_pton(AF_INET, "localhost", &server_addr.sin_addr);
		server_addr.sin_addr.s_addr = testip;
		if (bind(socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr))) {
			// socket bind failed
			int error = WSAGetLastError();
			logger_.Error("Socket binding failed : {}", error);
			ParseWSAError(error);

			return NetResult::BindFailed;
		}

		// run net thread
		logger_.Debug("[Winsock2] Launching network thread.");
		exit_flag_ = false;
		net_thread_ = new std::thread(&Winsock2UDPServer::NetworkThreadLoop, this);

		return NetResult::OK;
	}

	void Winsock2UDPServer::InternalClose()
	{
		exit_flag_ = true;
		if (net_thread_) {
			net_thread_->join();
			delete net_thread_;
			net_thread_ = nullptr;
			logger_.Debug("[Winsock2] Network thread cleaned up.");
		}

		if (socket_ != INVALID_SOCKET)
			closesocket(socket_);
		socket_ = INVALID_SOCKET;
		WSACleanup();
	}

	void Winsock2UDPServer::ParseWSAError(int wsa_error)
	{
		switch (wsa_error)
		{
		case WSASYSNOTREADY:
			// WSA startup error
			logger_.Error("Network sub-system is not ready. Check the Windows Socket dll.");
			break;

		case WSAENETDOWN:
			// socket : net sub system not available
			// bind : same, but not gonna happen
			logger_.Error("Network is down.");
			break;

		case WSAEMFILE:
			// socket : no more socket available
			logger_.Error("No more socket available.");
			break;

		case WSAENOBUFS:
			// socket : no buffer space available
			// bind : same
			logger_.Error("No buffer space available.");
			break;

		case WSAEADDRINUSE:
			// bind : addr already occupied
			logger_.Error("Current address already in use. Try with other port. Current port : {}", port());
			break;

		case WSAENETUNREACH:
			// sendto : network is unreachable
			logger_.Error("A socket operation was attempted to an unreachable network.");
			break;

		default:
			// unpredictable WSA error
			logger_.Error("Unchecked WSA Error and can't be handled.");
			break;
		}
	}

	void Winsock2UDPServer::NetworkThreadLoop()
	{
		static std::chrono::milliseconds delay(512);
		logger_.Debug("Internal network thread launched.");

		while (!exit_flag_) {
#ifdef DKVR_SYSTEM_ENABLE_FULL_THROTTLE
			if (delay.count() == kThreadDelayLowerLimit) {
				// full throttle mode
				using namespace std::chrono;

				bool first_yield = false;
				steady_clock::time_point yield_begin = steady_clock::now();
				while (true) {
					while (PeekSending() && PeekWritability())
						SendOneDatagram();

					if (PeekRecv()) {
						first_yield = true;
						HandleRecv();
					}
					else {
						if (first_yield) {
							first_yield = false;
							yield_begin = steady_clock::now();
						}

						milliseconds duration = duration_cast<milliseconds>(yield_begin - steady_clock::now());
						if (duration.count() > kYieldDurationLimit) {
							IncreaseDelay(delay);
							break;
						}
						std::this_thread::yield();
					}
				}
			}
			else 
#endif
			{
				// standard delay mode
				while (PeekSending() && PeekWritability()) {
					SendOneDatagram();
				}

				while (PeekRecv())
					HandleRecv();

				// throttle control
				int peak = 0;
				for (auto& item : arrivals_) {
					if (item.second > peak)
						peak = item.second;
					item.second = 0;
				}
				if (peak == 0)
					IncreaseDelay(delay);
				else if (peak >= 3)
					DecreaseDelay(delay);

				//logger_.Debug("Thread delay : {}ms", delay);
				std::this_thread::sleep_for(delay);
			}
		}
		logger_.Debug("Internal network thread closed.");
	}

	bool Winsock2UDPServer::PeekRecv()
	{
		constexpr TIMEVAL kTimeout{ 0, 0 };
		static fd_set fd_read;
		FD_ZERO(&fd_read);
		FD_SET(socket_, &fd_read);
		int result = select(NULL, &fd_read, nullptr, nullptr, &kTimeout);
		if (result == SOCKET_ERROR) {
			int error = WSAGetLastError();
			logger_.Debug("select function failed : {}", error);
			ParseWSAError(error);
		}

		return FD_ISSET(socket_, &fd_read);
	}

	void Winsock2UDPServer::HandleRecv()
	{
		constexpr int kBufSize = sizeof(Datagram::buffer);
		Datagram dgram{};
		sockaddr_in sender{};
		int sockaddr_size = sizeof(sockaddr_in);
		char* buffer = reinterpret_cast<char*>(&dgram.buffer);

		int res = recvfrom(socket_, buffer, kBufSize, 0, reinterpret_cast<sockaddr*>(&sender), &sockaddr_size);
		if (res == SOCKET_ERROR) {
			int error = WSAGetLastError();
			logger_.Error("Network recvfrom failed : {}", error);
			ParseWSAError(error);
			return;
		}
		dgram.address = sender.sin_addr.s_addr;
		PushReceived(dgram);

		auto iter = arrivals_.find(dgram.address);
		if (iter != arrivals_.end())
			iter->second++;
		else
			arrivals_.emplace(dgram.address, 1);
	}

	bool Winsock2UDPServer::PeekWritability()
	{
		constexpr TIMEVAL kTimeout{ 0 , 0 };
		static fd_set fd_write;
		FD_ZERO(&fd_write);
		FD_SET(socket_, &fd_write);
		select(NULL, nullptr, &fd_write, nullptr, &kTimeout);

		return FD_ISSET(socket_, &fd_write);
	}

	void Winsock2UDPServer::SendOneDatagram()
	{
		constexpr int kBufSize = sizeof(Datagram::buffer);
		constexpr int kSockaddrSize = sizeof(sockaddr_in);
		static sockaddr_in dst{
			.sin_family = AF_INET,
			.sin_port = htons(client_port())
		};

		Datagram dgram = PopSending();
		dst.sin_addr.s_addr = dgram.address;
		char* buffer = reinterpret_cast<char*>(&dgram.buffer);
		int len = dgram.buffer.length + 8;	// hmm... I don't like this

		int res = sendto(socket_, buffer, len, 0, reinterpret_cast<sockaddr*>(&dst), kSockaddrSize);
		if (res == SOCKET_ERROR) {
			int error = WSAGetLastError();
			logger_.Error("Network sendto failed : {}", error);
			ParseWSAError(error);
		}
	}

	static void IncreaseDelay(std::chrono::milliseconds& delay) {
		if (delay.count() < kThreadDelayUpperLimit)
			delay *= 2;
	}

	static void DecreaseDelay(std::chrono::milliseconds& delay) {
		if (delay.count() > kThreadDelayLowerLimit)
			delay /= 2;
	}

}	// namespace dkvr