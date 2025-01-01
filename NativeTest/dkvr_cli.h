#pragma once

#include <atomic>
#include <iostream>
#include <map>
#include <thread>
#include <tuple>
#include <vector>

#include "export/dkvr_host.h"
#include "include/util/thread_container.h"

namespace dkvr
{
    class DKVRCLI
    {
    public:
        DKVRCLI(int debug);
        ~DKVRCLI();
        void Run();
        void Command(const std::string& line);
        
        bool is_running() const { return is_running_; }

    private:
        void ParseInput(std::string line);
        bool TestArgsCount(int count) const { return args_.size() > count; }
        void UpdateImuRead();

    // list of DKVR functions
    private:
        void Help();
        void Exit();
        
        void List();
        void Status();
        void Locate();

        void Statistic();
        void Behavior();
        void Imu();
        void Show();
        void Show2();
        void Ypr();

        void Calib();
        void Save();
        void Load();

    private:
        // host control variables
        int debug_;
        std::atomic_bool is_running_ = false;
        DKVRHostHandle handle_ = nullptr;
        std::vector<std::string> args_;
        std::map<std::string, void(DKVRCLI::*)()> callbacks_;

        // imu reader variables
        ThreadContainer<DKVRCLI> imu_read_thread_;
        std::atomic_bool imu_read_active_ = false;
        std::atomic_int imu_read_target_ = -1;
        std::atomic_bool show[3]{};
        std::atomic_bool show2[3]{};
        std::atomic_bool ypr_export = false;

        // calibrator variables
        std::atomic_bool calibrator_active_ = false;
    };
}