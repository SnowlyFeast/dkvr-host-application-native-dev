#include "controller/instruction_dispatcher.h"

#include <thread>

#include "controller/instruction_set.h"

namespace dkvr {

	void InstructionDispatcher::Run()
	{
		if (thread_)
			return;

		logger_.Debug("Launching dispatcher thread.");
		exit_flag_ = false;
		thread_ = new std::thread(&InstructionDispatcher::DispatcherThreadLoop, this);
	}

	void InstructionDispatcher::Stop()
	{
		if (thread_) 
		{
			exit_flag_ = true;
			net_service_.RequestWakeup();
			thread_->join();
			delete thread_;
			thread_ = nullptr;
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

		// check header value
		if (inst.header != kHeaderValue) 
		{
			logger_.Error("Wrong header value({:X}) from {:d}.{:d}.{:d}.{:d}", inst.header, ip[0], ip[1], ip[2], ip[3]);
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
		target->set_recv_sequence_num(inst.sequence);

		// delegate to controller
		inst_handle_.Handle(target, inst);
	}

}	// namespace dkvr