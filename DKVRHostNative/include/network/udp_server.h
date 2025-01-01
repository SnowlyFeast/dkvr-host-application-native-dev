#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

#include "network/datagram.h"
#include "util/logger.h"

namespace dkvr 
{
    
    class UDPServer
    {
    public:
        static constexpr unsigned short kDefaultHostPort = 8899u;
        static constexpr unsigned short kDefaultClientPort = 8899u;

        enum class Status
        {
            InitRequired,
            InitFailed,
            StandBy,
            Running,
            Disposed,
            Error
        };

        UDPServer() : status_(Status::InitRequired), received_(), sending_(), mutex_(), convar_(), host_ip_(0), host_port_(kDefaultHostPort) { }
        virtual ~UDPServer() { }

        int Init();
        int Bind(unsigned long ip, unsigned short port);
        void Close();
        void Deinit();
        int PushSending(const Datagram& dgram);
        bool PeekReceived() const;
        bool WaitReceived() const;
        Datagram PopReceived();
        void Wakeup() { convar_.notify_all(); }

        Status         status() const       { return status_; }
        unsigned short client_port() const  { return kDefaultClientPort; }	// actually it's public const
        unsigned long  host_ip() const      { return host_ip_; }
        unsigned short host_port() const    { return host_port_; }

    protected:
        /**
         * @brief   Implementation specific.
         *          UDP server initializer.
         *          If initialization fails, DKVRHost instantiation is cancelled and Logger class is also unavailable.
         *          So in case of error, just throw @ref std::runtime_exception with details.
         * @return  always `return 0` (throw @c std::exception on fail)
         */
        virtual int InternalInit() = 0;

        /**
         * @brief   Implementation specific.
         *          Do UDP server binding and launching datagram send/recv handling thread.
         *          Zero @c host_ip() and @c host_port() means use the deafult value of it's implementation.
         *          This function is responsible for setting the final binded IP and port.
         * @return  `return 0` if no error occurs
         * @sa      UDPServer::set_host_ip(unsigned long)
         * @sa      UDPServer::set_host_port(unsigend short)
         */
        virtual int InternalBind() = 0;

        /**
         * @brief   Implementation specific.
         *          Stop the UDP server, this can be done with suspending or killing the thread.
         *          That doesn't mean full-cleanup of it's resources, recalling @c InternalBind() should re-run the UDP server.
         */
        virtual void InternalClose() = 0;

        /**
         * @brief   Implementation specific.
         *          Full-cleanup of it's network resources.
         *          Invoking @c UDPServer::Deinit() result irreverible shutdown of @c UDPServer
         */
        virtual void InternalDeinit() = 0;

        /**
         * @brief   Implementation specific.
         *          Handle the error state but this implementation is optional.
         *          This function will be called at @c UDPServer::Bind() if @c status_ is @c Status::Error
         * @return  `return 0` on success
         */
        virtual int InternalHandleError() { return 1; }


        /**
         * @brief   Check the queue waiting to be sent
         * @return  true if queue is not empty
         */
        bool PeekSending() const;

        /**
         * @brief   Pop and get front element of the sending queue.
         *          This function does not check the queue count, calling this function with empty queue result undefined behavior.
         *          Caller is responsible for checking availability.
         * @return  Front element of sending queue
         * @sa      UDPServer::PeekSending() const
         */
        Datagram PopSending();

        /**
         * @brief   Place @a dgram to sending queue.
         * @param dgram     @c Datagram to be sent
         */
        void PushReceived(const Datagram& dgram);

        /**
         * @brief   Set UDP server status to @c Status::Error
         *          Implemented logic should be suspended until @c Bind() is explicitly called again.
         *          Using this function is optional.
         * @sa      UDPServer::InternalHandleError()
         */
        void SetStatusToError() { status_ = Status::Error; }


        /**
         * @brief Set the binded host IPAdress visible from class outside
         * @param ip IPv4 address
         */
        void set_host_ip(unsigned long ip) { host_ip_ = ip; }
        /**
         * @brief Set the binded host port visible from class outside
         * @param port port number
         */
        void set_host_port(unsigned short port) { host_port_ = port; }

    private:
        UDPServer(const UDPServer&) = delete;
        UDPServer(UDPServer&&) = delete;
        void operator= (const UDPServer&) = delete;
        void operator= (UDPServer&&) = delete;

        Status status_;
        std::queue<Datagram> received_;
        std::queue<Datagram> sending_;
        mutable std::mutex mutex_;
        mutable std::condition_variable convar_;
        unsigned long host_ip_;
        unsigned short host_port_;

        Logger& logger_ = Logger::GetInstance();
    };

}	// namespace dkvr