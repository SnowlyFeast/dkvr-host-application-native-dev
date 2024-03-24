#include "pch.h"


#include "../DKVRHostNative/math/ellipsoid_estimator.h"
#include "../DKVRHostNative/math/matrix.h"
#include "../DKVRHostNative/math/vector.h"

#include "../DKVRHostNative/util/logger.h"

#include "../DKVRHostNative/network/network_service.h"
#include "../DKVRHostNative/tracker/tracker_provider.h"
#include "../DKVRHostNative/controller/instruction_set.h"
#include "../DKVRHostNative/controller/instruction_dispatcher.h"
#include "../DKVRHostNative/controller/tracker_updater.h"

// about to delete
#include "../DKVRHostNative/network/winsock2_udp_server.h"


using namespace std;
using namespace dkvr;

void Init();
void OpenDat(const string, vector<Vector3>&);
void SaveDat(const string, const vector<Vector3>&);
void PrintMatrix(const Vector&);
void PrintMatrix(const Matrix&);

bool exit_flag = false;


TEST(Struct, size_test)
{
	// set logger
	Logger& logger = Logger::GetInstance();
	logger << Logger::Mode::Echo << Logger::Level::Debug;
	logger.Debug("UnitTest Runner.");

	// componentes
	TrackerProvider& tk_provider = TrackerProvider::GetInstance();
	NetworkService& net_service = NetworkService::GetInstance();
	InstructionDispatcher inst_dispatcher;
	TrackerUpdater tk_updater;

	// init network
	bool net_result = net_service.Init();
	ASSERT_EQ(net_result, false);
	net_result = net_service.Run();
	ASSERT_EQ(net_result, false);

	// dispatcher and updater
	inst_dispatcher.Run();
	tk_updater.Run();


	uint32_t seq = 1;
	unsigned long ip = (192 | (168 << 8) | (0 << 16) | (11 << 24));
	while (true)
	{
		char k;
		cin >> k;

		if (k == 'c')
			break;

		if (k == 's')
		{
			Instruction inst{};
			inst.header = kHeaderValue;
			inst.opcode = static_cast<uint8_t>(Opcode::Status);
			inst.align = 0;
			inst.length = 0;
			inst.sequence = seq++;
			net_service.QueueSending(ip, inst);
		}

		if (k == 't')
		{
			AtomicTracker target = tk_provider.FindExistOrInsertNew(ip);
			cout << target->battery_perc();
		}

	}

	tk_updater.Stop();
	inst_dispatcher.Stop();
}


//TEST(Matrix, TestName) {
//	Init();
//
//	vector<Vector3> error;
//	OpenDat("c:/error.dat", error);
//
//	float mean = 0;
//	for (const Vector3& v : error)
//		mean += sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
//	mean /= error.size();
//	
//	float var = 0;
//	for (const Vector3& v : error)
//		var += powf(sqrtf(v.x * v.x + v.y * v.y + v.z * v.z) - mean, 2);
//	var /= error.size();
//
//	vector<Vector3> samples;
//	vector<Vector3> new_cal(samples.size());
//	OpenDat("c:/original.dat", samples);
//	
//	EllipsoidParameter param = EllipsoidEstimator::EstimateEllipsoid(samples, var);
//	Vector cv = param.GetCenterVector();
//	Matrix tm = param.GetTransformationMatrix();
//
//	PrintMatrix(cv);
//	PrintMatrix(tm);
//
//	for (auto iter = samples.begin(); iter != samples.end(); iter++) {
//		new_cal.push_back(tm * (iter->ToVector() - cv));
//	}
//
//	vector<Vector3> calib;
//	OpenDat("c:/calib.dat", calib);
//
//	float tot = 0;
//	cout << "Original\t\tPre Cal.\t\tNew Cal." << endl;
//	for (int i = 0; i < calib.size(); i++) {
//		Vector3 v_diff = calib[i] - new_cal[i];
//		float f_diff = sqrtf(powf(v_diff.x, 2) + powf(v_diff.y, 2) + powf(v_diff.z, 2));
//		tot += f_diff;
//		cout << f_diff << endl;
//	}
//	SaveDat("./calib2.dat", new_cal);
//}
//
//
//void Init()
//{
//	cout.precision(3);
//}
//
//void OpenDat(const string path, vector<Vector3>& dat) 
//{
//	ifstream f(path);
//	string buffer;
//	queue<string> buff;
//	while (getline(f, buffer)) {
//		stringstream ss(buffer);
//		string temp;
//		while (getline(ss, temp, ' '))
//			buff.push(temp);
//	}
//	while (buff.size() >= 3) {
//		float x = stof(buff.front());
//		buff.pop();
//		float y = stof(buff.front());
//		buff.pop();
//		float z = stof(buff.front());
//		buff.pop();
//		dat.push_back(Vector3{ x, y, z });
//	}
//	f.close();
//}
//
//void SaveDat(const string path, const vector<Vector3>& dat)
//{
//	ofstream f(path);
//	for (Vector3 s : dat) {
//		f << s.x << " " << s.y << " " << s.z << endl;
//	}
//	f.close();
//}
//
//void PrintMatrix(const Vector& vec) 
//{
//	std::cout << "<";
//	for (size_t i = 0; i < vec.size(); i++)
//		std::cout << vec[i] << "\t";
//	std::cout << ">" << std::endl;
//}
//
//void PrintMatrix(const Matrix& mat)
//{
//	for (unsigned long i = 0; i < mat.row(); i++) {
//		for (unsigned long j = 0; j < mat.column(); j++) {
//			std::cout << mat[i][j] << "\t";
//		}
//		std::cout << std::endl;
//	}
//}