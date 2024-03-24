#include "udp_server.h"

#include "net_result.h"

namespace dkvr {

	int UDPServer::Init()
	{
		int result = InternalInit();
		status_ = result ? Status::InitFailed : Status::StandBy;

		return result;
	}

	int UDPServer::Bind()
	{
		if (status_ == Status::Running)
			return NetResult::OK;
		else if (status_ == Status::InitRequired)
			return NetResult::InitRequired;
		else if (status_ == Status::InitFailed)
			return NetResult::InitFailed;
		else if (status_ == Status::Closed)
			return NetResult::ServerClosed;

		int result = InternalBind();
		if (result) {
			status_ = Status::Error;
		}
		else {
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
		status_ = Status::Closed;
	}

	int UDPServer::PushSending(const Datagram& dgram)
	{
		if (status_ != Status::Running)
			return NetResult::BindRequired;

		{
			std::lock_guard<std::mutex> lock(mutex_);
			sending_.push(dgram);
		}
		return NetResult::OK;
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