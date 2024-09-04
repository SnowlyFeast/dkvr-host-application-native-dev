#include "network/udp_server.h"

namespace dkvr 
{

	int UDPServer::Init()
	{
		int result = InternalInit();

		// InitFailed is meaningless as network service will just throw with non zero result
		status_ = result ? Status::InitFailed : Status::StandBy;	

		return result;
	}

	int UDPServer::Bind()
	{
		// check internal state
		switch (status_)
		{
		default:
		case UDPServer::Status::InitRequired:
		case UDPServer::Status::InitFailed:
		case UDPServer::Status::Closed:
		case UDPServer::Status::Error:
			logger_.Error("UDP Server is not on bind()-callable state : {}", static_cast<int>(status_));
			break;

		case UDPServer::Status::Running:
			logger_.Info("UDP Server is already running.");
			break;

		case UDPServer::Status::StandBy:
			break;
		}

		// run bind
        int result = InternalBind();
        if (result)
            status_ = Status::Error;
        else
        {
            status_ = Status::Running;
            exit_flag_ = false;
        }

		return result;
	}

	void UDPServer::Close()
	{
		exit_flag_ = true;
		convar_.notify_all();
		{
			std::lock_guard<std::mutex> lock(mutex_);
		}
		InternalClose();
		status_ = Status::StandBy;
	}

	void UDPServer::Deinit()
	{
		InternalDeinit();
		status_ = Status::Closed;
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