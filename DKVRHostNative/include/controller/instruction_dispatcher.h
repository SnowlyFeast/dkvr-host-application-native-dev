#pragma once

#include "controller/instruction_handler.h"
#include "instruction/instruction_format.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"
#include "util/thread_container.h"

namespace dkvr {

	class InstructionDispatcher
	{
	public:
		InstructionDispatcher(NetworkService& net_service, TrackerProvider& tk_provider);

		void Run();
		void Stop();

	private:
		void WaitReceiveAndDispatch();
		void Dispatch(unsigned long address, Instruction& inst);

		InstructionHandler inst_handler_;
		ThreadContainer<InstructionDispatcher> dispatcher_thread_;

		NetworkService& net_service_;
		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr