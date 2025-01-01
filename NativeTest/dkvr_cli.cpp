#include "dkvr_cli.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

namespace
{
    constexpr int kInvalidInt = -1;
    constexpr float kRadToDeg = (180.0f / 3.1415926535f);

    template <typename T>
    consteval T InvalidValue() { return T(-1); }

    template <typename T>
    T As(std::string str)
    {
        try
        {
            if (std::is_same_v<T, int>)                 return std::stoi(str);
            if (std::is_same_v<T, long>)                return std::stol(str);
            if (std::is_same_v<T, float>)               return std::stof(str);
            if (std::is_same_v<T, double>)              return std::stod(str);
            if (std::is_same_v<T, long double>)         return std::stold(str);
            if (std::is_same_v<T, unsigned long>)       return std::stoul(str);
            if (std::is_same_v<T, unsigned long long>)  return std::stoull(str);
            std::cout << "Unsupported parsing type." << std::endl;
        }
        catch (const std::invalid_argument& except)
        {
            std::cout << "Argument parsing failed." << std::endl;
        }

        return InvalidValue<T>();
    }

    int AsInt(std::string& str)
    {
        try
        {
            return std::stoi(str);
        }
        catch (const std::invalid_argument& except)
        {
            std::cout << "Argument parsing failed." << std::endl;
            return kInvalidInt;
        }
    }

    inline DKVRVector3 CrossProduct(const DKVRVector3& lhs, const DKVRVector3& rhs)
    {
        return DKVRVector3{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    DKVRVector3 EulerRodrigues(const DKVRQuaternion& quat, const DKVRVector3& vec)
    {
        DKVRVector3 vpart{ 2 * quat.x, 2 * quat.y, 2 * quat.z };
        DKVRVector3 cross1 = CrossProduct(vpart, vec);
        DKVRVector3 cross2 = CrossProduct(vpart, cross1);

        return DKVRVector3{ vec.x + cross1.x * quat.w + cross2.x, vec.y + cross1.y * quat.w + cross2.y, vec.z + cross1.z * quat.w + cross2.z };
    }

    /// Tait-Bryan angle ZYX intrinsic (yaw-pitch-roll)
    DKVRVector3 QuaternionToYpr(const DKVRQuaternion& quat)
    {
        // roll (x-axis rotation)
        float sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
        float cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
        float roll = std::atan2f(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = std::sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z));
        float cosp = std::sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z));
        float pitch = 2 * std::atan2f(sinp, cosp) - 3.1415926535f / 2;

        // yaw (z-axis rotation)
        float siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
        float cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
        float yaw = std::atan2f(siny_cosp, cosy_cosp);

        yaw *= kRadToDeg;
        pitch *= kRadToDeg;
        roll *= kRadToDeg;

        return DKVRVector3{ yaw, pitch, roll };
    }
}

namespace dkvr
{
    DKVRCLI::DKVRCLI(int debug) : debug_(debug), imu_read_thread_(*this)
    {
        // create callback map
        callbacks_.emplace("help", &DKVRCLI::Help);
        callbacks_.emplace("exit", &DKVRCLI::Exit);

        callbacks_.emplace("list", &DKVRCLI::List);
        callbacks_.emplace("status", &DKVRCLI::Status);
        callbacks_.emplace("locate", &DKVRCLI::Locate);

        if (debug)
        {
            callbacks_.emplace("statistic", &DKVRCLI::Statistic);
            callbacks_.emplace("behavior", &DKVRCLI::Behavior);
            callbacks_.emplace("imu", &DKVRCLI::Imu);
            callbacks_.emplace("show", &DKVRCLI::Show);
            callbacks_.emplace("show2", &DKVRCLI::Show2);
            callbacks_.emplace("ypr", &DKVRCLI::Ypr);

            callbacks_.emplace("calib", &DKVRCLI::Calib);
            callbacks_.emplace("save", &DKVRCLI::Save);
            callbacks_.emplace("load", &DKVRCLI::Load);
        }
        
        // attach callback to thread runner
        imu_read_thread_ += &DKVRCLI::UpdateImuRead;
        imu_read_thread_.Run();
    }

    DKVRCLI::~DKVRCLI()
    {
        imu_read_thread_.Stop();
        if (is_running_)
            Exit();
    }

    void DKVRCLI::Run()
    {
        // already running
        if (is_running_) return;

        // check header and dll version
        int version_ok;
        dkvrAssertVersion(DKVR_HOST_EXPORTED_HEADER_VER, &version_ok);
        if (!version_ok)
        {
            int dll_version;
            dkvrGetVersion(&dll_version);
            std::cout << "Host version is not compatible with current dll version." << std::endl;
            std::cout << "Host version : " << (int)DKVR_HOST_EXPORTED_HEADER_VER << std::endl;
            std::cout << "DLL  version : " << (int)dll_version << std::endl;
            return;
        }

        // create instance and assert handle
        char init_result[255];
        dkvrCreateInstance(&handle_, init_result, sizeof init_result);
        if (handle_ == nullptr)
        {
            std::cout << "Failed to create DKVRHost instance." << std::endl;
            return;
        }

        // setup logger
        dkvrLoggerSetLoggerOutput(handle_, std::cout);

        // run instance and assert
        int running;
        DKVRAddress addr{ 0 };
        dkvrRunHost(handle_, addr);
        dkvrIsRunning(handle_, &running);
        is_running_ = running;
        if (!is_running_)
        {
            std::cout << "Failed to start DKVRHost." << std::endl;
            dkvrDeleteInstance(&handle_);
            return;
        }
        
        std::cout << "DKVRHost launched." << std::endl;
        is_running_ = true;
    }

    void DKVRCLI::Command(const std::string& line)
    {
        if (!is_running_)
        {
            std::cout << "DKVRHost is not running." << std::endl;
            return;
        }
        ParseInput(line);
    }

    void DKVRCLI::ParseInput(std::string line)
    {
        std::istringstream iss(line);
        std::string buffer;

        args_.clear();
        while (getline(iss, buffer, ' ')) {
            args_.push_back(buffer);
        }

        if (args_.size() == 0) return;

        auto iter = callbacks_.find(args_[0]);
        if (iter == callbacks_.end())
        {
            std::cout << "Unknown command. Type 'help' for command list." << std::endl;
            return;
        }

        (this->*(iter->second))();
    }

    void DKVRCLI::UpdateImuRead()
    {
        // invalid target or calibrator is running
        if (imu_read_target_ == -1 || calibrator_active_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            return;
        }

        DKVRVector3 vec3[5]{};
        DKVRQuaternion quat;
        if (show[0])    dkvrTrackerGetRawGyro(handle_, imu_read_target_, &vec3[0]);
        if (show[1])    dkvrTrackerGetRawAccel(handle_, imu_read_target_, &vec3[1]);
        if (show[2])    dkvrTrackerGetRawMag(handle_, imu_read_target_, &vec3[2]);

        if (show2[0])   dkvrTrackerGetOrientation(handle_, imu_read_target_, &quat);
        if (show2[1])   dkvrTrackerGetLinearAcceleration(handle_, imu_read_target_, &vec3[3]);
        if (show2[2])   dkvrTrackerGetMagneticDisturbance(handle_, imu_read_target_, &vec3[4]);

        int line_count = 0;
        std::vector<std::string> lines;

        if (show[0])    // gyro
        {
            DKVRVector3 vec;
            dkvrTrackerGetRawGyro(handle_, imu_read_target_, &vec);
            line_count++;
            std::stringstream ss;
            ss  << std::setprecision(3) << std::fixed << "Gyr : "
                << std::setw(8) << vec.x * kRadToDeg
                << std::setw(8) << vec.y * kRadToDeg
                << std::setw(8) << vec.z * kRadToDeg;
            lines.push_back(ss.str());
        }
        if (show[1])    // accel
        {
            DKVRVector3 vec;
            dkvrTrackerGetRawAccel(handle_, imu_read_target_, &vec);
            line_count++;
            std::stringstream ss;
            ss  << std::setprecision(3) << std::fixed << "Acc : "
                << std::setw(8) << vec.x
                << std::setw(8) << vec.y
                << std::setw(8) << vec.z;
            lines.push_back(ss.str());
        }
        if (show[2])    // mag
        {
            DKVRVector3 vec;
            dkvrTrackerGetRawMag(handle_, imu_read_target_, &vec);
            line_count++;
            std::stringstream ss;
            ss  << std::setprecision(3) << std::fixed << "Mag : "
                << std::setw(8) << vec.x
                << std::setw(8) << vec.y
                << std::setw(8) << vec.z;
            lines.push_back(ss.str());
        }
        if (show2[0])   // orientation
        {
            DKVRQuaternion quat;
            dkvrTrackerGetOrientation(handle_, imu_read_target_, &quat);
            line_count++;
            std::stringstream ss;

            DKVRVector3 ypr = QuaternionToYpr(quat);

            ss  << std::setprecision(3) << std::fixed << "Orientation  : "
                << std::setw(8) << ypr.x
                << std::setw(8) << ypr.y
                << std::setw(8) << ypr.z;
            lines.push_back(ss.str());
        }
        if (show2[1])   // linear acceleration
        {
            DKVRVector3 vec;
            dkvrTrackerGetLinearAcceleration(handle_, imu_read_target_, &vec);
            line_count++;
            std::stringstream ss;
            ss  << std::setprecision(3) << std::fixed << "Linear Accel : "
                << std::setw(8) << vec.x
                << std::setw(8) << vec.y
                << std::setw(8) << vec.z;
            lines.push_back(ss.str());
        }
        if (show2[2])   // magnetic disturbance
        {
            DKVRVector3 vec;
            dkvrTrackerGetMagneticDisturbance(handle_, imu_read_target_, &vec);
            line_count++;
            std::stringstream ss;
            ss  << std::setprecision(3) << std::fixed << "Mag Disturb. : "
                << std::setw(8) << vec.x
                << std::setw(8) << vec.y
                << std::setw(8) << vec.z;
            lines.push_back(ss.str());
        }

        // print to top of screen
        if (line_count > 0)
        {
            std::cout << "\033[s"       // save cursor
                      << "\033[1;1H";   // move cursor to top left
            for (int i = 0; i < line_count; i++)
            {
                std::cout << "\033[2K"; // erase line
                std::cout << lines[i];
                std::cout << "\033[1B"; // cursor down
                std::cout << '\r';      // carriage return
            }
            std::cout << "\033[u";      // restore cursor
        }

        // export ypr if enabled
        if (ypr_export)
        {   
            std::ofstream fout("./ypr.dat", std::ios::trunc);
            if (fout.is_open())
            {
                DKVRVector3 forward = EulerRodrigues(quat, DKVRVector3{ 1, 0, 0 });
                DKVRVector3 head = EulerRodrigues(quat, DKVRVector3{ 0, 0, -1 });

                fout << forward.x << " " << forward.y << " " << forward.z << " " << head.x << " " << head.y << " " << head.z;
                fout.close();
            }
        }
        
        // delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // DKVR functions
    void DKVRCLI::Help()
    {
        std::cout << "----------------------Available command----------------------" << '\n';
        std::cout << "help" << '\n';
        std::cout << "exit" << '\n';
        std::cout << '\n';

        std::cout << "list" << '\n';
        std::cout << "status [index?]" << '\n';
        std::cout << "locate [index]" << '\n';

        if (debug_)
        {
            std::cout << '\n';
            std::cout << "statistic [index?]" << '\n';
            std::cout << "behavior [index] [led? active? raw? nominal?]" << '\n';
            std::cout << "imu read [index]" << '\n';
            std::cout << "imu stop" << '\n';
            std::cout << "show  [g? a? m?]" << '\n';
            std::cout << "show2 [o? a? m?]" << '\n';
            std::cout << "ypr [0/1]" << '\n';
            std::cout << '\n';

            std::cout << "calib status" << '\n';
            std::cout << "calib begin [index]" << '\n';
            std::cout << "calib continue [async?]" << '\n';
            std::cout << "calib abort" << '\n';
            std::cout << "calib perc" << '\n';
            std::cout << '\n';

            std::cout << "save [index] [calib] [filename]" << '\n';
            std::cout << "load [index] [calib] [filename]" << '\n';
        }
        
        std::cout << "-------------------------------------------------------------" << std::endl;
    }

    void DKVRCLI::Exit()
    {
        // stop IMU reader thread
        imu_read_thread_.Stop();

        // clean up
        dkvrStopHost(handle_);
        dkvrDeleteInstance(&handle_);
        is_running_ = false;
    }

    void DKVRCLI::List()
    {
        using namespace std;

        int count = 0;
        dkvrTrackerGetCount(handle_, &count);

        cout    << setw(3)  << "# " << "|"
                << setw(14) << "Connection  " << "|"
                << setw(17) << "Address     " << "|"
                << setw(5)  << "RTT " << "|"
                << setw(10)  << "Behavior " << "|"
                << setw(30) << "Tracker Name         "
                << '\n';
        cout << "------------------------------------------------------------------------------------" << endl;

        for (int i = 0; i < count; i++)
        {
            char name[52]{};
            int connection;
            std::string connection_str;
            unsigned long address;
            int rtt;
            int behavior[4]{};

            dkvrTrackerGetName(handle_, i, name, sizeof name);
            dkvrTrackerGetConnectionStatus(handle_, i, &connection);
            dkvrTrackerGetAddress(handle_, i, &address);
            dkvrTrackerGetRtt(handle_, i, &rtt);
            dkvrTrackerGetBehaviorLed(handle_, i, behavior + 0);
            dkvrTrackerGetBehaviorActive(handle_, i, behavior + 1);
            dkvrTrackerGetBehaviorRaw(handle_, i, behavior + 2);
            dkvrTrackerGetBehaviorNominal(handle_, i, behavior + 3);

            if (connection == DKVRConnectionStatus::Connected)
                connection_str = "Connected";
            else if (connection == DKVRConnectionStatus::Handshaked)
                connection_str = "Handshaked";
            else
                connection_str = "Disconnected";

            unsigned char* ptr = reinterpret_cast<unsigned char*>(&address);
            stringstream ss;
            ss << (int)ptr[0] << "." << (int)ptr[1] << "." << (int)ptr[2] << "." << (int)ptr[3];

            cout << setw(2)  << i << " |"
                 << setw(13) << connection_str << " |"
                 << setw(16) << ss.str() << " |"
                 << setw(4)  << rtt << " |"
                 << "  ["    << (behavior[0] ? "L" : " ")
                             << (behavior[1] ? "A" : " ")
                             << (behavior[2] ? "R" : " ")
                             << (behavior[3] ? "N" : " ") << "]  |"
                 << setw((30 + std::strlen(name)) / 2) << name
                 << endl;
        } // for
        cout << "----------------------------------------END-----------------------------------------" << endl;
    }

    void DKVRCLI::Status()
    {
        int target, count;
        if (TestArgsCount(1))
        {
            // specific
            target = AsInt(args_[1]);
            if (target == kInvalidInt) return;
            count = target + 1;
        }
        else
        {
            // all
            target = 0;
            dkvrTrackerGetCount(handle_, &count);
        }

        std::cout   << std::setw(2) << "# "     << "| "
                    << std::setw(5) << "init "  << "| "
                    << std::setw(5) << "bat% "  << "\n"
                    << "----------------"
                    << std::endl;
        for (int i = target; i < count; i++)
        {
            int init_result, battery_perc;
            dkvrTrackerGetInitResult(handle_, i, &init_result);
            dkvrTrackerGetBatteryPerc(handle_, i, &battery_perc);

            std::cout   << std::setw(2) << i << "| "
                        << "0x" << std::setw(2) << std::setfill('0') << std::hex << init_result << " | "
                        << std::setfill(' ') << std::dec << std::setw(3) << battery_perc << "%"
                        << std::endl;
        }

    }

    void DKVRCLI::Locate()
    {
        if (!TestArgsCount(1))
        {
            std::cout << "Missing argument : target index" << std::endl;
            return;
        }
        int target = AsInt(args_[1]);
        if (target == -1) return;
        dkvrTrackerRequestLocate(handle_, target);
    }

    void DKVRCLI::Statistic()
    {
        int target, count;
        if (TestArgsCount(1))
        {
            // specific
            target = AsInt(args_[1]);
            if (target == kInvalidInt) return;
            count = target + 1;
        }
        else
        {
            // all
            target = 0;
            dkvrTrackerGetCount(handle_, &count);
        }

        std::cout   << std::setw(2) << "# " << "| "
                    << std::setw(11) << "Exec. Time " << "| "
                    << std::setw(11) << "INT Miss % " << "| "
                    << std::setw(11) << "IMU Miss % " << "\n"
                    << "-----------------------------------------"
                    << std::endl;
        for (int i = target; i < count; i++)
        {
            int execution_time, interrupt_miss, imu_miss;
            dkvrTrackerGetExecutionTime(handle_, target, &execution_time);
            dkvrTrackerGetInterruptMissRate(handle_, target, &interrupt_miss);
            dkvrTrackerGetImuMissRate(handle_, target, &imu_miss);

            std::cout   << std::setw(2) << i << "| "
                        << std::setw(6) << execution_time << " ms  " << "| "
                        << std::setw(6) << interrupt_miss << " %   " << "| "
                        << std::setw(6) << imu_miss       << " %   " << "\n"
                        << std::endl;
        }
    }

    void DKVRCLI::Behavior()
    {
        if (!TestArgsCount(1))
        {
            std::cout << "Missing argument : target index" << std::endl;
            return;
        }

        int target = AsInt(args_[1]);
        if (target == kInvalidInt) return;

        bool led = false;
        bool active = false;
        bool raw = false;
        bool nominal = false;

        for (int i = 2; i < args_.size(); i++)
        {
            std::string& arg = args_[i];
            if      (!arg.compare("led"))     led     = true;
            else if (!arg.compare("active"))  active  = true;
            else if (!arg.compare("raw"))     raw     = true;
            else if (!arg.compare("nominal")) nominal = true;
            else if (!arg.compare("all"))
            {
                led = true;
                active = true;
                raw = true;
                nominal = true;
            }
        }

        dkvrTrackerSetBehaviorLed(handle_, target, led);
        dkvrTrackerSetBehaviorActive(handle_, target, active);
        dkvrTrackerSetBehaviorRaw(handle_, target, raw);
        dkvrTrackerSetBehaviorNominal(handle_, target, nominal);

        std::stringstream ss;
        if (!led && !active && !raw && !nominal)    // nothing active
            ss << "[ None ]";
        else
        {
            ss << "[ ";
            if (led)     ss << "LED ";
            if (active)  ss << "Active ";
            if (raw)     ss << "Raw ";
            if (nominal) ss << "Nominal ";
            ss << "]";
        }

        std::cout << "Setting " << target << "th tracker's behavior to " << ss.str() << std::endl;
    }

    void DKVRCLI::Imu()
    {
        if (!TestArgsCount(1))
        {
            std::cout << "Missing argument : imu action (read / stop)" << std::endl;
            return;
        }

        // read target tracker data
        if (!args_[1].compare("read"))
        {
            if (!TestArgsCount(2))
            {
                std::cout << "Missing 2nd argument : target index." << std::endl;
                return;
            }
            int target = AsInt(args_[2]);
            
            int count;
            dkvrTrackerGetCount(handle_, &count);
            if (target >= count)
            {
                std::cout << "Index out of range." << std::endl;
                return;
            }

            imu_read_active_ = true;
            imu_read_target_ = target;
        }
        // stop reading
        else if (!args_[1].compare("stop"))
        {
            imu_read_active_ = false;
            imu_read_target_ = -1;
        }
        // wrong arg
        else
        {
            std::cout << "Unknown first arg. only 'read' and 'stop' allowed." << std::endl;
        }
    }

    void DKVRCLI::Show()
    {
        std::fill_n(show, 3, false);
        for (int i = 1; i < args_.size(); i++)
        {
            if      (!args_[i].compare("g")) show[0] = true;
            else if (!args_[i].compare("a")) show[1] = true;
            else if (!args_[i].compare("m")) show[2] = true;
        }
    }

    void DKVRCLI::Show2()
    {
        std::fill_n(show2, 3, false);
        for (int i = 1; i < args_.size(); i++)
        {
            if      (!args_[i].compare("o")) show2[0] = true;
            else if (!args_[i].compare("a")) show2[1] = true;
            else if (!args_[i].compare("m")) show2[2] = true;
        }
    }

    void DKVRCLI::Ypr()
    {
        if (!TestArgsCount(1))
        {
            std::cout << "Missing argument : enable (0 / 1)" << std::endl;
            return;
        }
        int val = AsInt(args_[1]);
        if (val == kInvalidInt) return;
        ypr_export = val;
    }

    void DKVRCLI::Calib()
    {
        if (!TestArgsCount(1))
        {
            // missing arg
            std::cout << "Missing 1st argument : status / begin / continue / abort" << std::endl;
            return;
        }
        else
        {
            if (!args_[1].compare("status"))
            {
                int status = 0;
                char buffer[64]{};
                dkvrCalibratorGetStatus(handle_, &status);
                dkvrCalibratorGetStatusString(handle_, buffer, sizeof(buffer));
                std::cout << "Calibrator Status : " << buffer << std::endl;

                if (status != DKVRCalibratorStatus::Idle)
                {
                    int target = 0;
                    dkvrCalibratorGetCurrentTarget(handle_, &target);
                    std::cout << "Current calibration target : " << target << std::endl;
                }

                if (status == DKVRCalibratorStatus::StandBy)
                {
                    dkvrCalibratorGetSampleTypeString(handle_, buffer, sizeof(buffer));
                    std::cout << "Required sample type : " << buffer << std::endl;
                }
            }
            else if (!args_[1].compare("begin"))
            {
                if (!TestArgsCount(2))
                {
                    std::cout << "Missing 2nd argument  : target index" << std::endl;
                    return;
                }
                
                // parse
                int target = AsInt(args_[2]);
                if (target == kInvalidInt) return;
                
                // check range
                int count;
                dkvrTrackerGetCount(handle_, &count);
                if (target >= count)
                {
                    std::cout << "Index out of range." << std::endl;
                    return;
                }
                
                dkvrCalibratorBeginWith(handle_, target);
            }
            else if (!args_[1].compare("continue"))
            {
                dkvrCalibratorContinue(handle_);

                if (TestArgsCount(2) && !args_[2].compare("async"))
                {
                    // non blocking mode
                    return;
                }
                else
                {
                    // block until calibration finish
                    calibrator_active_ = true;
                    int status;
                    dkvrCalibratorGetStatus(handle_, &status);
                    while (status == DKVRCalibratorStatus::Recording)
                    {
                        int progress;
                        dkvrCalibratorGetProgress(handle_, &progress);
                        std::cout << "\033[s"       // save cursor
                                  << "\033[1;1H"    // move cursor to top left
                                  << "\033[K"       // erase line
                                  << "Calibrator progress : " << progress << "%      "
                                  << "\033[u";      // restore cursor
                        std::this_thread::yield();
                        dkvrCalibratorGetStatus(handle_, &status);
                    }
                    calibrator_active_ = false;
                }
            }
            else if (!args_[1].compare("abort"))
            {
                dkvrCalibratorAbort(handle_);
            }
            else if (!args_[1].compare("perc"))
            {
                int progress;
                dkvrCalibratorGetProgress(handle_, &progress);
                std::cout << "Calibrator progress : " << progress << "%" << std::endl;
            }
            else
            {
                std::cout << "Unknown calibrator command." << std::endl;
                return;
            }
        }
    }

    // TODO: remake saving and loading seq
    void DKVRCLI::Save()
    {
        if (!TestArgsCount(3))
        {
            std::cout << "Missing argument." << std::endl;
            return;
        }

        int target = AsInt(args_[1]);
        if (target == kInvalidInt) return;
        
        if (!args_[2].compare("calib"))
        {
            DKVRCalibration calib;
            dkvrTrackerGetCalibration(handle_, target, &calib);
            const void* ptr = &calib;
            int count = sizeof DKVRCalibration;

            if (ptr && count)
            {
                // print it's calibration values and write to file
                std::cout << std::scientific;
                std::cout << "Gyro Calibration : ";
                for (int i = 0; i < 12; i++) std::cout << calib.gyr_transform[i] << " ";
                std::cout << std::endl;
                std::cout << "Accel Calibration : ";
                for (int i = 0; i < 12; i++) std::cout << calib.acc_transform[i] << " ";
                std::cout << std::endl;
                std::cout << "Mag Calibration : ";
                for (int i = 0; i < 12; i++) std::cout << calib.mag_transform[i] << " ";
                std::cout << std::endl;
                std::cout << "Noise Variance : ";
                for (int i = 0; i < 9; i++) std::cout << calib.noise_variance[i] << " ";
                std::cout << std::endl;
                std::cout << std::fixed;

                std::ofstream output(args_[3], std::ios::binary | std::ios::trunc);
                if (output.is_open())
                {
                    output.write(reinterpret_cast<const char*>(ptr), count);
                    output.close();
                    std::cout << "Calibration struct saved." << std::endl;
                }
                else
                {
                    std::cout << "Output file open failed." << std::endl;
                }
            }
        }
    }

    void DKVRCLI::Load()
    {
        if (!TestArgsCount(3))
        {
            std::cout << "Missing argument." << std::endl;
            return;
        }

        int target = AsInt(args_[1]);
        if (target == kInvalidInt) return;

        std::ifstream input(args_[3], std::ios::binary);
        if (input.is_open())
        {
            input.seekg(0, input.end);
            int size = input.tellg();
            input.seekg(0, input.beg);

            char* buffer = new char[size];
            input.read(buffer, size);

            if (!args_[2].compare("calib"))
            {
                if (size == sizeof DKVRCalibration)
                {
                    dkvrTrackerSetCalibration(handle_, target, reinterpret_cast<DKVRCalibration*>(buffer));
                    std::cout << "Calibration struct loaded." << std::endl;
                }
                else
                {
                    std::cout << "filesize is not matching with sizeof Calibration struct." << std::endl;
                }
            }

            delete[] buffer;
        }
        else
        {
            std::cout << "Input file open failed." << std::endl;
        }
    }

}   // namespace dkvr