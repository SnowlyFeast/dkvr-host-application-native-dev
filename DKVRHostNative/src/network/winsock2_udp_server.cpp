#ifdef _WIN32

#include "network/winsock2_udp_server.h"

#include <chrono>
#include <sstream>
#include <thread>

#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

namespace dkvr 
{

    namespace 
    {
        constexpr TIMEVAL kTimeout{ 0, 0 };
    }

    Winsock2UDPServer::Winsock2UDPServer() :
        winsock_thread_(*this),
        wsa_data_{},
        socket_(INVALID_SOCKET),
        binding_ip_(0) 
    {
        winsock_thread_ += &Winsock2UDPServer::SendAndRecvMessage;
    }

    Winsock2UDPServer::~Winsock2UDPServer()
    {
        InternalClose();
    }

    int Winsock2UDPServer::InternalInit()
    {
        // WSA start up
        int wsa_result = WSAStartup(MAKEWORD(2, 2), &wsa_data_);
        if (wsa_result != 0) {
            // WSAStartup failed
            WSACleanup();
            
            // you cannot use logger nor ParseWSAError func here, just throw exception
            throw std::runtime_error(Logger::FormatString("WSA startup failed : WSAError {}", wsa_result));
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
            if (!getaddrinfo(hostname, std::to_string(host_port()).c_str(), &hints, &result)) {
                for (addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
                    if (ptr->ai_family == AF_INET) {
                        unsigned long ip = reinterpret_cast<sockaddr_in*>(ptr->ai_addr)->sin_addr.s_addr;
                        unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
                        // will be logged if init successful
                        logger_.Info("Host ip address is {:d}.{:d}.{:d}.{:d}", ptr[0], ptr[1], ptr[2], ptr[3]);
                        binding_ip_ = ip;
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

            // same, just throw exception
            throw std::runtime_error(Logger::FormatString("Socket creation failed : WSAError {}", error));
        }

        return 0;
    }

    int Winsock2UDPServer::InternalBind()
    {
        sockaddr_in server_addr{
            .sin_family = AF_INET,
            .sin_port = 0
        };

        // use specific IP and port if provided
        if (host_ip())
            server_addr.sin_addr.s_addr = host_ip();
        else
            server_addr.sin_addr.s_addr = binding_ip_;

        if (host_port())
            server_addr.sin_port = htons(host_port());
        else
            server_addr.sin_port = htons(kDefaultHostPort);
        

        if (bind(socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr))) {
            // socket bind failed
            int error = WSAGetLastError();
            logger_.Error("Socket binding failed : {}", error);
            ParseWSAError(error);

            return 1;
        }

        // log host ip and port
        set_host_ip(server_addr.sin_addr.s_addr);
        set_host_port(ntohs(server_addr.sin_port));
        
        unsigned char* ptr = reinterpret_cast<unsigned char*>(&server_addr.sin_addr.s_addr);
        logger_.Info("Host is binded to {}.{}.{}.{}:{}", ptr[0], ptr[1], ptr[2], ptr[3], host_port());

        // run net thread
        winsock_thread_.Run();
        logger_.Debug("Winsock2 network thread launched.");

        return 0;
    }

    void Winsock2UDPServer::InternalClose()
    {
        winsock_thread_.Stop();
        logger_.Debug("Winsock2 network thread closed.");
    }

    void Winsock2UDPServer::InternalDeinit()
    {
        InternalClose();
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
            logger_.Error("Current address already in use. Try with other port. Current port : {}", host_port());
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

    void Winsock2UDPServer::SendAndRecvMessage()
    {
        while (PeekSending() && PeekWritability())
            SendOneDatagram();

        while (PeekRecv())
            HandleRecv();

        std::this_thread::yield();
    }

    bool Winsock2UDPServer::PeekRecv()
    {
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
        Datagram dgram{};
        sockaddr_in sender{};
        int sockaddr_size = sizeof sockaddr_in;
        char* buffer = reinterpret_cast<char*>(&dgram.buffer);

        int res = recvfrom(socket_, buffer, sizeof Datagram::buffer, 0, reinterpret_cast<sockaddr*>(&sender), &sockaddr_size);
        if (res == SOCKET_ERROR) 
        {
            int error = WSAGetLastError();
            logger_.Error("Network recvfrom failed : {}", error);
            ParseWSAError(error);
            return;
        }
        dgram.address = sender.sin_addr.s_addr;
        PushReceived(dgram);
    }

    bool Winsock2UDPServer::PeekWritability() const
    {
        static fd_set fd_write;
        FD_ZERO(&fd_write);
        FD_SET(socket_, &fd_write);
        select(NULL, nullptr, &fd_write, nullptr, &kTimeout);

        return FD_ISSET(socket_, &fd_write);
    }

    void Winsock2UDPServer::SendOneDatagram()
    {
        static sockaddr_in dst{
            .sin_family = AF_INET,
            .sin_port = htons(client_port())
        };

        Datagram dgram = PopSending();
        dst.sin_addr.s_addr = dgram.address;
        char* buffer = reinterpret_cast<char*>(&dgram.buffer);
        int len = dgram.buffer.length + 8;	// hmm... I don't like this
                                            // dgram.buffer.length is 'payload' length and 8 is header length

        int res = sendto(socket_, buffer, len, 0, reinterpret_cast<sockaddr*>(&dst), sizeof sockaddr_in);
        if (res == SOCKET_ERROR) 
        {
            int error = WSAGetLastError();
            logger_.Error("Network sendto failed : {}", error);
            ParseWSAError(error);
        }
    }

}	// namespace dkvr

#endif  // #ifdef _WIN32