#include "network/network_service.h"

#include <cstring>
#include <stdexcept>

#ifdef DKVR_SYSTEM_BIG_ENDIAN
#   include <algorithm>
#endif
#ifdef _WIN32
#	include "network/winsock2_udp_server.h"
#else
#	error "Include UDP Server header here."
#endif

namespace dkvr {

#ifdef DKVR_SYSTEM_BIG_ENDIAN
    static void BigEndianBitConversion(Instruction& inst);
#endif
    static bool IsPowerOf2(uint8_t num);

    NetworkService::NetworkService() :
#ifdef _WIN32
        udp_(std::make_unique<Winsock2UDPServer>())
#endif
    {
        if (udp_->Init())
            throw std::runtime_error("UDP server init failed");
    }

    NetworkService::~NetworkService()
    {
        udp_->Deinit();
    }

    bool NetworkService::Run(unsigned short port)
    {
        udp_->set_port(port);
        int result = udp_->Bind();
        if (result) 
        {
            logger_.Error("[Network Service] Internal UDP Server bind failed.");
            return true;
        }
        return false;
    }

    void NetworkService::Stop()
    {
        udp_->Close();
    }

    unsigned long NetworkService::WaitAndPopReceived(Instruction& out)
    {
        if (udp_->WaitReceived()) 
        {
            Datagram&& dgram = udp_->PopReceived();
            memcpy_s(&out, sizeof(Instruction), &dgram.buffer, sizeof(Instruction));
#ifdef DKVR_SYSTEM_BIG_ENDIAN
            BigEndianBitConversion(out);
#endif
            return dgram.address;
        }
        else
            return 0;
    }

    void NetworkService::Send(unsigned long address, Instruction& inst)
    {
#ifdef DKVR_SYSTEM_BIG_ENDIAN
        BigEndianBitConversion(inst);
#endif
        if (udp_->PushSending(Datagram{ address, inst }))
            logger_.Error("[Network Service] Instruction queuing failed : internal UDP Server not bounded.");
    }

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

    bool IsPowerOf2(uint8_t num)
    {
        return !(num & (num - 1));
    }

}   // namespace dkvr