#include "controller/instruction_handler.h"

#include "instruction/instruction_set.h"

#include "tracker/tracker.h"
#include "tracker/tracker_data.h"
#include "tracker/tracker_debug.h"

#include "util/hash.h"

namespace dkvr
{

    namespace
    {
        constexpr uint8_t kCalibTransformSize = sizeof TrackerCalibration::gyr_transform;
        constexpr uint8_t kCalibNoiseVarSize = sizeof TrackerCalibration::noise_variance;
    }

    void InstructionHandler::Handle(Tracker* target, Instruction& inst)
    {
        switch (Opcode(inst.opcode))
        {
            // network
        case Opcode::Handshake1:
            Handshake1(target, inst); break;

        case Opcode::Handshake2:
            Handshake2(target, inst); break;

        case Opcode::Heartbeat:
            Heartbeat(target, inst); break;

        case Opcode::Ping:
            Ping(target, inst); break;

        case Opcode::Pong:
            Pong(target, inst); break;

            // miscellanous
        case Opcode::Locate:
            Locate(target, inst); break;

        case Opcode::ClientName:
            ClientName(target, inst); break;

            // configuration
        case Opcode::Behavior:
            Behavior(target, inst); break;

        case Opcode::GyrTransform:
            GyrTransform(target, inst); break;

        case Opcode::AccTransform:
            AccTransform(target, inst); break;

        case Opcode::MagTransform:
            MagTransform(target, inst); break;

        case Opcode::NoiseVariance:
            NoiseVariance(target, inst); break;


            // data transfer
        case Opcode::Status:
            Status(target, inst); break;

        case Opcode::Raw:
            Raw(target, inst); break;

        case Opcode::Nominal:
            Nominal(target, inst); break;

        case Opcode::Statistic:
            Statistic(target, inst); break;

        case Opcode::Debug:
            Debug(target, inst); break;


            // unknown opcode
        default:
        {
            unsigned long ip = target->address();
            unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
            logger_.Error(
                "Unknown instruction(0x{:x}) received from {:d}.{:d}.{:d}.{:d}",
                inst.opcode,
                ptr[0], ptr[1], ptr[2], ptr[3]
            );
            break;
        }
        }	// switch case
    }

    void InstructionHandler::Handshake1(Tracker* target, Instruction& inst)
    {
        if (target->IsDisconnected()) 
        {
            target->SetHandshaked();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
            unsigned long ip = target->address();
            unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
            logger_.Debug("Tracker handshaked (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
        }
    }

    void InstructionHandler::Handshake2(Tracker* target, Instruction& inst)
    {
        // host side opcode
        logger_.Debug("Host-side opcode(Handshake2) received.");
    }

    void InstructionHandler::Heartbeat(Tracker* target, Instruction& inst)
    {
        if (target->IsHandshaked())
        {
            target->SetConnected();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
            unsigned long ip = target->address();
            unsigned char* ptr = reinterpret_cast<unsigned char*>(&ip);
            logger_.Debug("Tracker connected (ip {:d}.{:d}.{:d}.{:d})", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
        }
        target->UpdateHeartbeatRecv();
    }

    void InstructionHandler::Ping(Tracker* target, Instruction& inst)
    {
        // host side opcdoe
        logger_.Debug("Host-side opcode(Ping) received.");
    }

    void InstructionHandler::Pong(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
            target->UpdateRtt();
    }

    void InstructionHandler::Locate(Tracker* target, Instruction& inst)
    {
        // host side opcode
        logger_.Debug("Host-side opcode(Locate) received.");
    }

    void InstructionHandler::ClientName(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
            target->set_name(reinterpret_cast<char*>(inst.payload));
    }

    void InstructionHandler::Behavior(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            if (inst.payload[0].uchar[0] == target->behavior_encoded())
                target->SetBehaviorSynced();
        }
    }

    void InstructionHandler::GyrTransform(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            uint8_t hash = Hash::Pearson(kCalibTransformSize, target->calibration_cref().gyr_transform);
            if (hash == inst.payload[0].uchar[0])
                target->SetGyrTransformSynced();
        }
    }

    void InstructionHandler::AccTransform(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            uint8_t hash = Hash::Pearson(kCalibTransformSize, target->calibration_cref().acc_transform);
            if (hash == inst.payload[0].uchar[0])
                target->SetAccTransformSynced();
        }
    }

    void InstructionHandler::MagTransform(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            uint8_t hash = Hash::Pearson(kCalibTransformSize, target->calibration_cref().mag_transform);
            if (hash == inst.payload[0].uchar[0])
                target->SetMagTransformSynced();
        }
    }

    void InstructionHandler::NoiseVariance(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            uint8_t hash = Hash::Pearson(kCalibNoiseVarSize, target->calibration_cref().noise_variance);
            if (hash == inst.payload[0].uchar[0])
                target->SetNoiseVarianceSynced();
        }
    }

    void InstructionHandler::Status(Tracker* target, Instruction& inst)
    {
        TrackerStatus* status = reinterpret_cast<TrackerStatus*>(inst.payload);
        target->set_tracker_status(*status);
    }

    void InstructionHandler::Raw(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected()) 
        {
            RawDataSet* data = reinterpret_cast<RawDataSet*>(inst.payload);
            target->set_raw_data(*data);
        }
    }

    void InstructionHandler::Nominal(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected()) 
        {
            NominalDataSet* data = reinterpret_cast<NominalDataSet*>(inst.payload);
            target->set_nominal_data(*data);
        }
    }

    void InstructionHandler::Statistic(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            TrackerStatistic* statistic = reinterpret_cast<TrackerStatistic*>(&inst.payload);
            target->set_tracker_statistic(*statistic);
        }
    }

    void InstructionHandler::Debug(Tracker* target, Instruction& inst)
    {
        if (target->IsConnected())
        {
            TrackerDebug* debug = reinterpret_cast<TrackerDebug*>(&inst.payload);
            logger_.Debug("{} (dkvr_err {:#04x} / timestamp {}) from {}.", debug->msg, debug->dkvr_err, debug->timestamp, target->name());
        }
    }

}