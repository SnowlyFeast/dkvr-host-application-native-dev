#pragma once

#include "controller.h"

namespace dkvr {

	class NetworkingController : Controller
	{
	public:
		void Handle(Tracker* target, Instruction& inst) override;

	private:
		void Handshake1(Tracker* target, Instruction& inst);
		void Handshake2(Tracker* target, Instruction& inst);
		void Heartbeat(Tracker* target, Instruction& inst);
		void Ping(Tracker* target, Instruction& inst);
		void Pong(Tracker* target, Instruction& inst);
	};

}	// namespace dkvr