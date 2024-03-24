#include "con_network.h"

#include <chrono>

#include "instruction_set.h"

namespace dkvr {

	void NetworkingController::Handle(Tracker* target, Instruction& inst)
	{
		switch (Opcode(inst.opcode))
		{
		case Opcode::Handshake1:
			Handshake1(target, inst);
			break;

		case Opcode::Handshake2:
			Handshake2(target, inst);
			break;

		case Opcode::Heartbeat:
			Heartbeat(target, inst);
			break;

		case Opcode::Ping:
			Ping(target, inst);
			break;

		case Opcode::Pong:
			Pong(target, inst);
			break;

		default:
			break;
		}
	}

	void NetworkingController::Handshake1(Tracker* target, Instruction& inst)
	{
		if (target->IsDisconnected()) {
			target->SetHandshaked();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
			unsigned long ip = target->address();
			unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
			logger_.Debug("Tracker handshaked (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
		}
	}

	void NetworkingController::Handshake2(Tracker* target, Instruction& inst)
	{
		// host side opcode
		logger_.Debug("Host-side opcode(Handshake2) received.");
	}

	void NetworkingController::Heartbeat(Tracker* target, Instruction& inst)
	{
		if (target->IsHandshaked()) {
			target->SetConnected();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
			unsigned long ip = target->address();
			unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
			logger_.Debug("Tracker connected (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
		}
		target->UpdateHeartbeatRecv();
	}

	void NetworkingController::Ping(Tracker* target, Instruction& inst)
	{
		// host side opcdoe
		logger_.Debug("Host-side opcode(Ping) received.");
	}

	void NetworkingController::Pong(Tracker* target, Instruction& inst)
	{
		target->UpdateRtt();
	}

}	// namespace dkvr