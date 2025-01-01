#include "controller/instruction_dispatcher.h"

#include "instruction/instruction_set.h"

namespace dkvr {

	InstructionDispatcher::InstructionDispatcher(NetworkService& net_service, TrackerProvider& tk_provider): 
		inst_handler_(),
		dispatcher_thread_(*this),
		net_service_(net_service), 
		tk_provider_(tk_provider) 
	{ 
		dispatcher_thread_ += &InstructionDispatcher::WaitReceiveAndDispatch;
	}

	void InstructionDispatcher::Run()
	{
		dispatcher_thread_.Run();
		logger_.Debug("Dispatcher thread launched.");

	}

	void InstructionDispatcher::Stop()
	{
        // because WaitAndPopReceived needs wakeup,
        // asynchronously call Stop() and send wakeup signal
		dispatcher_thread_.StopAsync();
        net_service_.RequestWakeup();
		while (dispatcher_thread_.IsRunning());	// just spin lock cuz there's nothing to do
        logger_.Debug("Dispatcher thread closed.");
	}

	void InstructionDispatcher::WaitReceiveAndDispatch()
	{
		unsigned long address;
		Instruction inst;
		if (net_service_.WaitAndPopReceived(address, inst))
			Dispatch(address, inst);
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
		inst_handler_.Handle(target, inst);

		// update recv_sequence only on connected status
		if (target->IsConnected())
			target->set_recv_sequence_num(inst.sequence);
	}

}	// namespace dkvr