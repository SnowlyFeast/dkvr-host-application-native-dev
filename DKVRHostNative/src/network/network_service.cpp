#include "network/network_service.h"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#	include "network/winsock2_udp_server.h"
#else
#	error "Include UDP Server header here."
#endif


// try check big-endian
#if defined(__has_include) && __has_include(<endian.h>)
#   include <endian.h>
#endif
#if defined(__BYTE_ORDER__)&&(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)|| \
    defined(__BYTE_ORDER  )&&(__BYTE_ORDER   == __ORDER_BIG_ENDIAN  )
#   define DKVR_SYSTEM_BIG_ENDIAN
#endif

// ignore DKVR_SYSTEM_BIG_ENDIAN option on well-known system
#if defined(_WIN32)||defined(__APPLE__)
#   ifdef DKVR_SYSTEM_BIG_ENDIAN
#       undef DKVR_SYSTEM_BIG_ENDIAN
#   endif
#endif

#ifdef DKVR_SYSTEM_BIG_ENDIAN
#   include <algorithm>
#endif

namespace dkvr 
{

    namespace
    {
        constexpr std::chrono::milliseconds kWatchdogThreadDelay(1000);

        void DoBitConversionIfRequired(Instruction& inst)
        {
#ifdef DKVR_SYSTEM_BIG_ENDIAN
        void BigEndianBitConversion(Instruction& inst)
        {
            // bit reverse for sequence number
            char* seq_ptr = reinterpret_cast<char*>(&inst.sequence);
            std::reverse(seq_ptr, seq_ptr + sizeof(uint32_t));

            // bit reverse for payload
            // align should be larger than 1 and power of 2 
            char* payload_ptr = reinterpret_cast<char*>(&inst.payload);
            if (inst.align > 1 && IsPowerOf2(inst.align))
            {
                for (int i = 0; i < inst.length; i += inst.align)
                    std::reverse(payload_ptr + i, payload_ptr + i + inst.align);
            }
        }
#endif
        bool IsPowerOf2(uint8_t num) { return !(num & (num - 1)); }
    }

    NetworkService::NetworkService() :
        udp_(
#ifdef _WIN32
            std::make_unique<Winsock2UDPServer>()
#endif
        ),
        watchdog_thread_(*this)
    {
        if (udp_->Init())
            throw std::runtime_error("UDP server init failed with unknown reason.");

        watchdog_thread_ += &NetworkService::CheckAndRepairService;
    }

    NetworkService::~NetworkService()
    {
        udp_->Deinit();
    }

    bool NetworkService::Run(unsigned long ip, unsigned short port)
    {
        int result = udp_->Bind(ip, port);

        if (result)
        {
            logger_.Error("[Network Service] Internal UDP Server bind failed.");
            return true;
        }

        watchdog_thread_.Run();
        logger_.Debug("UDP server watchdog launched.");
        return false;
    }

    void NetworkService::Stop()
    {
        udp_->Close();
        watchdog_thread_.Stop();
        logger_.Debug("UDP server watchdog closed.");
    }

    bool NetworkService::WaitAndPopReceived(unsigned long& address_out, Instruction& inst_out)
    {
        if (udp_->WaitReceived())
        {
            Datagram dgram = udp_->PopReceived();
            address_out = dgram.address;
            inst_out = dgram.buffer;
            DoBitConversionIfRequired(inst_out);
            return true;
        }
        return false;
    }

    void NetworkService::Send(unsigned long address, Instruction& inst)
    {
        Datagram dgram{ address, inst };
        DoBitConversionIfRequired(dgram.buffer);
        if (udp_->PushSending(dgram))
            logger_.Error("[Network Service] Instruction queuing failed : internal UDP Server not binded.");
    }

    // UDP server watchdog
    void NetworkService::CheckAndRepairService()
    {
        while (udp_->status() == UDPServer::Status::Running)
            std::this_thread::sleep_for(kWatchdogThreadDelay);

        // if udp server closed by UDPServer::Close(), it's status should be Status::StandBy
        switch (udp_->status())
        {
        case UDPServer::Status::StandBy:
            // server is closed normally
            break;

        case UDPServer::Status::Error:
        {
            // try recover error, error handling logic is embeded in UDPServer::Bind()
            int result = udp_->Bind(udp_->host_ip(), udp_->host_port());
            if (result)
            {
                logger_.Error("UDP server error recovery failed. retrying after 5 seconds : {}", result);
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
            else
            {
                logger_.Debug("UDP server rebinded.");
            }
            break;
        }

            // theses are not gonna happen
        default:
        //case UDPServer::Status::InitRequired:
        //case UDPServer::Status::InitFailed:
        //case UDPServer::Status::Running:
        //case UDPServer::Status::Disposed:
            break;
        }
    }

}   // namespace dkvr