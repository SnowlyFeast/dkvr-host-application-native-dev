#include "network/udp_server.h"

namespace dkvr 
{

	int UDPServer::Init()
	{
		int result = InternalInit();

		// InitFailed status is meaningless as NetworkService will just throw if result is non-zero
		status_ = result ? Status::InitFailed : Status::StandBy;	

		return result;
	}

	int UDPServer::Bind(unsigned long ip, unsigned short port)
	{
		// try handle the error
		if (status_ == UDPServer::Status::Error)
			if (!InternalHandleError())
				status_ = UDPServer::Status::StandBy;

		// check internal state
		switch (status_)
		{
		default:
		case UDPServer::Status::InitRequired:
		case UDPServer::Status::InitFailed:
		case UDPServer::Status::Disposed:
		case UDPServer::Status::Error:
			logger_.Error("UDP Server is not on bind() callable state : {}", static_cast<int>(status_));
			return -1;

		case UDPServer::Status::Running:
			logger_.Info("UDP Server is already running.");
			return 0;

		case UDPServer::Status::StandBy:
			break;
		}

		// run bind
		set_host_ip(ip);
		set_host_port(port);
        int result = InternalBind();
		if (result)
		{
			status_ = Status::Error;
			logger_.Error("UDP Server binding failed.");
		}
        else
        {
            status_ = Status::Running;
        }

		return result;
	}

	void UDPServer::Close()
	{
		convar_.notify_all();
		{
			// actually volatile keyword is not needed but just a little protection for compiler optimization
			// tested at Compiler Explorer for MSVC v19 with '-O2' and GCC 14.2 with '-O3'
			// generate same code with or without volatile
			volatile std::lock_guard<std::mutex> lock(mutex_);
		}
		InternalClose();
		status_ = Status::StandBy;
	}

	void UDPServer::Deinit()
	{
		InternalDeinit();
		status_ = Status::Disposed;
	}

	int UDPServer::PushSending(const Datagram& dgram)
	{
		if (status_ != Status::Running)
		{
			logger_.Error("Call Bind() before push any dgram.");	// it's implementation problem
			return 1;
		}

		{
			std::lock_guard<std::mutex> lock(mutex_);
			sending_.push(dgram);
		}
		return 0;
	}

	bool UDPServer::PeekSending() const
	{
		bool available;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			available = !sending_.empty();
		}
		return available;
	}

	Datagram UDPServer::PopSending()
	{
		Datagram result;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			result = sending_.front();
			sending_.pop();
		}
		return result;
	}

	void UDPServer::PushReceived(const Datagram& dgram)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			received_.push(dgram);
		}
		convar_.notify_one();
	}

	bool UDPServer::PeekReceived() const
	{
		bool available;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			available = !received_.empty();
		}
		return available;
	}

	bool UDPServer::WaitReceived() const
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (received_.empty()) {
			convar_.wait(lock);
			return !received_.empty();
		}
		return true;
	}

	Datagram UDPServer::PopReceived()
	{
		Datagram result;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			result = received_.front();
			received_.pop();
		}
		return result;
	}

}	// namespace dkvr