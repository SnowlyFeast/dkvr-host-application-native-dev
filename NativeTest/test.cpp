#include "pch.h"

#include "export/dkvr_host.h"

// main component
DKVRHostHandle handle = nullptr;
char global_buffer[2048];
std::atomic_bool exit_flag = false;

// imu reader
std::atomic_bool imu_read_exit = false;
std::unique_ptr<std::thread> imu_read_thread(nullptr);
std::atomic_bool show[4] = { true, true, true, true };
std::atomic_bool imu_ypr = false;

// protos
void LoggerCheckingThreadLoop();
void ParseKeyInput();
void ReadIMUThreadLoop(int target, int interval);

TEST(Struct, size_test)
{
	// check header and dll version
	int assertion;
	dkvrAssertVersion(DKVR_HOST_EXPORTED_HEADER_VER, &assertion);
	ASSERT_TRUE(assertion);
	
	// create instance and assert handle
	dkvrCreateInstance(&handle);
	ASSERT_NE(handle, nullptr);

	// run instance and assert
	dkvrRunHost(handle);
	dkvrIsRunning(handle, &assertion);
	ASSERT_TRUE(assertion);

	// run local thread
	std::unique_ptr<std::thread> thread_console = std::make_unique<std::thread>(LoggerCheckingThreadLoop);


	// key input parser
	while (!exit_flag)
	{
		ParseKeyInput();
	}

	// close threads
	exit_flag = true;
	thread_console->join();

	// clean up
	dkvrStopHost(handle);
	dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
	std::cout << global_buffer;

	dkvrDeleteInstance(&handle);
}

void LoggerCheckingThreadLoop()
{
	while (!exit_flag)
	{
		int count = 0;
		dkvrLoggerGetUncheckCount(handle, &count);

		if (count)
		{
			dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
			std::cout << '\r' << global_buffer << "> ";
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

static bool compare(std::string& target, const char* ref)
{
	return !target.compare(ref);
}

static std::vector<std::string> split(std::string str, char Delimiter) {
	std::istringstream iss(str);
	std::string buffer;

	std::vector<std::string> result;

	while (getline(iss, buffer, Delimiter)) {
		result.push_back(buffer);
	}
	return result;
}

#define ASSERT_ARG(arg, ss)	if (arg.size() < ss) { std::cout << "Missing arguments." << std::endl; return; }
#define PARSE_STR(result, str, func)							\
do {															\
	try {														\
		result = func(str);										\
	}															\
	catch (std::invalid_argument except)						\
	{															\
		std::cout << "argument parsing failed." << std::endl;	\
	}															\
} while(0)

void ParseKeyInput()
{
	std::cout << "> ";
	std::string str;
	std::getline(std::cin, str, '\n');
	if (str.size() == 0)
		return;

	std::vector<std::string> arg = split(str, ' ');
	std::string cmd = arg[0];

	if (compare(cmd, "help"))
	{
		std::cout << "----------------------Available command----------------------" << std::endl;
		std::cout << "exit" << std::endl;
		std::cout << "list" << std::endl;
		std::cout << "behavior [index] [value]" << std::endl;
		std::cout << "imu [index] [interval]" << std::endl;
		std::cout << "imustop" << std::endl;
		std::cout << "show [q g a m]" << std::endl;
		std::cout << "ypr [0/1]" << std::endl;
		std::cout << std::endl;

		std::cout << "static" << std::endl;
		std::cout << "static update" << std::endl;
		std::cout << "status" << std::endl;
		std::cout << "status update" << std::endl;
		std::cout << "locate [index]" << std::endl;
		std::cout << std::endl;

		std::cout << "calib status" << std::endl;
		std::cout << "calib begin [index]" << std::endl;
		std::cout << "calib continue" << std::endl;
		std::cout << "calib abort" << std::endl;
		std::cout << std::endl;

		std::cout << "save calib [index] [filename]" << std::endl;
		std::cout << "load calib [index] [filename]" << std::endl;
		std::cout << "-------------------------------------------------------------" << std::endl;
	}
	else if (compare(cmd, "exit"))
	{
		imu_read_exit = true;
		if (imu_read_thread)
			imu_read_thread->join();
		exit_flag = true;
	}
	else if (compare(cmd, "list"))
	{
		int count = 0;
		dkvrTrackerGetCount(handle, &count);

		std::cout << "idx\taddress\t\tconnection\tbehavior" << std::endl;
		std::cout << "-------------------------------------------------------------" << std::endl;
		for (int i = 0; i < count; i++)
		{
			long address = 0;
			dkvrTrackerGetAddress(handle, i, &address);

			int con = 0;
			dkvrTrackerGetConnectionStatus(handle, i, &con);
			std::string con_str = con == 0 ? "Disconnected" : (con == 1 ? "Handshaked" : "Connected");

			int active = 0;
			int raw = 0;
			int led = 0;
			dkvrTrackerGetActive(handle, i, &active);
			dkvrTrackerGetRaw(handle, i, &raw);
			dkvrTrackerGetLed(handle, i, &led);
			int behavior = (led << 2) | (raw << 1) | active;

			unsigned char* ptr = reinterpret_cast<unsigned char*>(&address);
			std::cout << i << '\t' << (int)ptr[0] << "." << (int)ptr[1] << "." << (int)ptr[2] << "." << (int)ptr[3] << '\t' << con_str << '\t'
				<< "[" << (active ? "active|" : "") << (raw ? "raw|" : "") << (led ? "led" : "") << "]"
				<< std::endl;
		}
	}
	else if (compare(cmd, "behavior"))
	{
		int target, behavior;
		ASSERT_ARG(arg, 3);
		PARSE_STR(target, arg[1], stoi);
		PARSE_STR(behavior, arg[2], stoi);

		int active = behavior & 0b001;
		int raw = behavior & 0b010;
		int led = behavior & 0b100;
		dkvrTrackerSetActive(handle, target, active);
		dkvrTrackerSetRaw(handle, target, raw);
		dkvrTrackerSetLed(handle, target, led);
		std::cout << "Setting " << target << "th tracker's behavior to " << behavior << "." << std::endl;
	}
	else if (compare(cmd, "imu"))
	{
		ASSERT_ARG(arg, 3);

		if (imu_read_thread)
		{
			std::cout << "IMU reader thread already running." << std::endl;
			return;
		}

		int target, interval;
		PARSE_STR(target, arg[1], stoi);
		PARSE_STR(interval, arg[2], stoi);

		imu_read_exit = false;
		imu_read_thread = std::make_unique<std::thread>(ReadIMUThreadLoop, target, interval);
	}
	else if (compare(cmd, "imustop"))
	{
		imu_read_exit = true;
		if (imu_read_thread)
			imu_read_thread->join();
		imu_read_thread.reset();
	}
	else if (compare(cmd, "show"))
	{
		bool q = 0;
		bool g = 0;
		bool a = 0;
		bool m = 0;
		for (int i = 1; i < arg.size(); i++)
		{
			if (compare(arg[i], "q"))
				q = true;
			else if (compare(arg[i], "g"))
				g = true;
			else if (compare(arg[i], "a"))
				a = true;
			else if (compare(arg[i], "m"))
				m = true;
		}
		show[0] = q;
		show[1] = g;
		show[2] = a;
		show[3] = m;

		std::cout << "Now IMU read shows " << (show[0] ? "Q" : "") << (show[1] ? "G" : "") << (show[2] ? "A" : "") << (show[3] ? "M" : "") << std::endl;
	}
	else if (compare(cmd, "ypr"))
	{
		int enable;
		ASSERT_ARG(arg, 2);
		PARSE_STR(enable, arg[1], stoi);

		imu_ypr = enable ? true : false;
		std::cout << "YPR setting updated." << std::endl;
	}
	else if (compare(cmd, "static"))
	{
		int count = 0;
		dkvrTrackerGetCount(handle, &count);
		if (arg.size() >= 2 && compare(arg[1], "update"))
		{
			for (int i = 0; i < count; i++)
				dkvrTrackerRequestStatistic(handle, i);
			std::cout << "Tracker statistic update requested." << std::endl;
		}
		else
		{
			std::cout << "idx\texecution time" << std::endl;
			std::cout << "-------------------------------------------------------------" << std::endl;
			for (int i = 0; i < count; i++)
			{
				int exec_time;
				dkvrTrackerGetExecutionTime(handle, i, &exec_time);
				std::cout << i << '\t' << exec_time << " ms" << std::endl;
			}
		}
	}
	else if (compare(cmd, "status"))
	{
		int count = 0;
		dkvrTrackerGetCount(handle, &count);
		if (arg.size() >= 2 && compare(arg[1], "update"))
		{
			for (int i = 0; i < count; i++)
				dkvrTrackerRequestStatus(handle, i);
			std::cout << "Tracker status update requested." << std::endl;
		}
		else
		{
			std::cout << "idx\tinit\tbattery %" << std::endl;
			std::cout << "-------------------------------------------------------------" << std::endl;
			for (int i = 0; i < count; i++)
			{
				int init, battery_perc;
				dkvrTrackerGetInitResult(handle, i, &init);
				dkvrTrackerGetBatteryPerc(handle, i, &battery_perc);
				std::cout << i << '\t' << std::hex << (bool)init << '\t' << std::dec << battery_perc << std::endl;
			}
		}
	}
	else if (compare(cmd, "locate"))
	{
		int target;
		ASSERT_ARG(arg, 2);
		PARSE_STR(target, arg[1], stoi);

		dkvrTrackerRequestLocate(handle, target);
	}
	else if (compare(cmd, "calib"))
	{
		ASSERT_ARG(arg, 2);

		std::string& second = arg[1];
		if (compare(second, "status"))
		{
			int status = 0;
			char buffer[64]{};
			dkvrCalibratorGetStatus(handle, &status);
			dkvrCalibratorGetStatusString(handle, buffer, sizeof(buffer));
			std::cout << "Calibrator Status : " << buffer << std::endl;

			if (status != DKVRCalibratorStatus::Idle)
			{
				int target = 0;
				dkvrCalibratorGetCurrentTarget(handle, &target);
				std::cout << "Current calibration target : " << target << std::endl;
			}

			if (status == DKVRCalibratorStatus::StandBy)
			{
				dkvrCalibratorGetRequiredSampleTypeString(handle, buffer, sizeof(buffer));
				std::cout << "Required sample type : " << buffer << std::endl;
			}
		}
		else if (compare(second, "begin"))
		{
			int target;
			ASSERT_ARG(arg, 3);
			PARSE_STR(target, arg[2], stoi);

			dkvrCalibratorBeginWith(handle, target);
		}
		else if (compare(second, "continue"))
		{
			dkvrCalibratorContinue(handle);
		}
		else if (compare(second, "abort"))
		{
			dkvrCalibratorAbort(handle);
		}
		else if (compare(second, "perc"))
		{
			int progress;
			dkvrCalibratorGetProgress(handle, &progress);
			std::cout << "Calibrator progress : " << progress << "%" << std::endl;
		}
		else
		{
			std::cout << "Unknown calibrator command." << std::endl;
		}
	}
	else if (compare(cmd, "save")) 
	{
		int target;
		ASSERT_ARG(arg, 4);
		PARSE_STR(target, arg[2], stoi);

		const void* ptr = nullptr;
		size_t count = 0;
		if (compare(arg[1], "calib"))
		{
			static DKVRCalibration calib;
			dkvrTrackerGetCalibration(handle, target, &calib);
			ptr = &calib;
			count = sizeof DKVRCalibration;

			std::cout << "Gyro Calibration : ";
			for (int i = 0; i < 12; i++) std::cout << calib.gyr_transform[i] << " " ;
			std::cout << std::endl;
			std::cout << "Accel Calibration : ";
			for (int i = 0; i < 12; i++) std::cout << calib.acc_transform[i] << " ";
			std::cout << std::endl;
			std::cout << "Mag Calibration : ";
			for (int i = 0; i < 12; i++) std::cout << calib.mag_transform[i] << " ";
			std::cout << std::endl;
			std::cout << "Noise Variance : ";
			for (int i = 0; i < 3; i++) std::cout << calib.gyr_noise_var[i] << " ";
			for (int i = 0; i < 3; i++) std::cout << calib.acc_noise_var[i] << " ";
			for (int i = 0; i < 3; i++) std::cout << calib.mag_noise_var[i] << " ";
			std::cout << std::endl;
		}

		if (ptr && count) {
			std::ofstream output(arg[3], std::ios::binary | std::ios::trunc);
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
		else
		{
			std::cout << "Save target is not specified." << std::endl;
		}
		
	}
	else if (compare(cmd, "load"))
	{
		int target;
		ASSERT_ARG(arg, 4);
		PARSE_STR(target, arg[2], stoi);

		std::ifstream input(arg[3], std::ios::binary);
		if (input.is_open())
		{
			input.seekg(0, input.end);
			int size = input.tellg();
			input.seekg(0, input.beg);

			char* buffer = new char[size];
			input.read(buffer, size);

			if (compare(arg[1], "calib"))
			{
				if (size == sizeof DKVRCalibration)
				{
					dkvrTrackerSetCalibration(handle, target, reinterpret_cast<DKVRCalibration*>(buffer));
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
	else
	{
		std::cout << "unknown command" << std::endl;
	}
}

DKVRVector3 CrossProduct(DKVRVector3 lhs, DKVRVector3 rhs)
{
	return DKVRVector3{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
}

DKVRVector3 EulerRodrigues(DKVRQuaternion quat, DKVRVector3 vec)
{
	DKVRVector3 vpart{ 2 * quat.x, 2 * quat.y, 2 * quat.z };
	DKVRVector3 cross1 = CrossProduct(vpart, vec);
	DKVRVector3 cross2 = CrossProduct(vpart, cross1);

	return DKVRVector3{ vec.x + cross1.x * quat.w + cross2.x, vec.y + cross1.y * quat.w + cross2.y, vec.z + cross1.z * quat.w + cross2.z };
}

void ReadIMUThreadLoop(int target, int interval)
{
	while (!imu_read_exit)
	{
		DKVRQuaternion quat{};
		DKVRVector3 gyro{}, accel{}, mag{};
		dkvrTrackerGetOrientation(handle, target, &quat);
		dkvrTrackerGetGyro(handle, target, &gyro);
		dkvrTrackerGetAccel(handle, target, &accel);
		dkvrTrackerGetMag(handle, target, &mag);

		if (imu_ypr)
		{
			// roll (x-axis rotation)
			double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
			double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
			double roll = std::atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			double sinp = std::sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z));
			double cosp = std::sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z));
			double pitch = 2 * std::atan2(sinp, cosp) - 3.1415926535f / 2;

			// yaw (z-axis rotation)
			double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
			double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
			double yaw = std::atan2(siny_cosp, cosy_cosp);	

			constexpr double kRadToDeg = (180.0 / 3.1415926535);
			yaw *= kRadToDeg;
			pitch *= kRadToDeg;
			roll *= kRadToDeg;
			std::cout << std::setprecision(3) << std::fixed << "YPR : (" << yaw << ", " << pitch << ", " << roll << ")" << std::endl;

			std::ofstream fout("./ypr.dat", std::ios::trunc);
			if (fout.is_open())
			{
				DKVRVector3 forward = EulerRodrigues(quat, DKVRVector3{ 1, 0, 0 });
				DKVRVector3 head = EulerRodrigues(quat, DKVRVector3{ 0, 0, -1 });

				fout << forward.x << " " << forward.y << " " << forward.z << " " << head.x << " " << head.y << " " << head.z;
				fout.close();
			}
		}
		else 
		{
			if (show[0])
				std::cout << std::setprecision(3) << std::fixed << "Quat  : " << quat.x << "\t" << quat.y << "\t" << quat.z << "\t" << quat.w << std::endl;

			if (show[1])
				std::cout << std::setprecision(3) << std::fixed << "Gyro  : " << gyro.x << "\t" << gyro.y << "\t" << gyro.z << std::endl;

			if (show[2])
				std::cout << std::setprecision(3) << std::fixed << "Accel : " << accel.x << "\t" << accel.y << "\t" << accel.z << std::endl;

			if (show[3])
				std::cout << std::setprecision(3) << std::fixed << "Mag   : " << mag.x << "\t" << mag.y << "\t" << mag.z << std::endl;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(interval));
	}
}