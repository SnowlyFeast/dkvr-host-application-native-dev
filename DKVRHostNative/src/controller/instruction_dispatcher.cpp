#include "controller/instruction_dispatcher.h"

#include <thread>

#include "instruction/instruction_set.h"

namespace dkvr {

	InstructionDispatcher::InstructionDispatcher(NetworkService& net_service, TrackerProvider& tk_provider)
		: thread_ptr_(nullptr), exit_flag_(false), inst_handle_(), net_service_(net_service), tk_provider_(tk_provider) { }

	void InstructionDispatcher::Run()
	{
		if (thread_ptr_)
			return;

		logger_.Debug("Launching dispatcher thread.");
		exit_flag_ = false;
		thread_ptr_ = std::make_unique<std::thread>(&InstructionDispatcher::DispatcherThreadLoop, this);
	}

	void InstructionDispatcher::Stop()
	{
		if (thread_ptr_) 
		{
			exit_flag_ = true;
			net_service_.RequestWakeup();
			thread_ptr_->join();
			thread_ptr_.reset();
			logger_.Debug("Dispatcher thread successfully closed.");
		}
	}

	void InstructionDispatcher::DispatcherThreadLoop()
	{
		Instruction inst{ };
		while (!exit_flag_) 
		{
			unsigned long address = net_service_.WaitAndPopReceived(inst);
			if (address)
				Dispatch(address, inst);
		}
	}

	void InstructionDispatcher::Dispatch(unsigned long address, Instruction& inst)
	{
		unsigned char* ip = reinterpret_cast<unsigned char*>(&address);

		// check opener value
		if (inst.opener != kOpenerValue) 
		{
			logger_.Error("Wrong header value({:X}) from {:d}.{:d}.{:d}.{:d}", inst.opener, ip[0], ip[1], ip[2], ip[3]);
			return;
		}

		// discard late datagram
		AtomicTracker target = tk_provider_.FindExistOrInsertNew(address);
		if (target->recv_sequence_num() > inst.sequence) 
		{
			logger_.Debug(
				"Late datagram discarded from {:d}.{:d}.{:d}.{:d}, current : {} / recieved : {}",
				ip[0], ip[1], ip[2], ip[3],
				target->recv_sequence_num(),
				inst.sequence
			);
			return;
		}

		// delegate to controller
		inst_handle_.Handle(target, inst);

		// update recv_sequence only on connected status
		if (target->IsConnected())
			target->set_recv_sequence_num(inst.sequence);
	}

}	// namespace dkvr