//
// Created by steve on 6/14/19.
//


#include <iostream>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "AWF.h"

#include "../AuxiliaryTool/UwbTools.h"
#include "../AuxiliaryTool/UwbTools.cpp"

#include "../AuxiliaryTool/ImuTools.h"
#include "../AuxiliaryTool/ImuTools.cpp"

#include "BayesFilter/KalmanFilter/IMUESKF.h"
#include "BayesFilter/KalmanFilter/IMUESKF.cpp"

#include <chrono>

/**
 * @brief Get current data in second[s].
 * @return
 */
double get_time() {
//	return double(std::chrono::system_clock::now().time_since_epoch().count()) / 1e9;
	return std::chrono::duration<double>(
			std::chrono::system_clock::now().time_since_epoch()
	).count();
}


int main(int argc, char *argv[]) {
	std::cout.precision(30);

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	double start_read_time = get_time();

	std::string dir_name = "/home/steve/Data/VehicleUWBINS/0003/";

	AWF::FileReader imu_file(dir_name + "HEAD.data"),
			uwb_file(dir_name + "uwb_data.csv"),
			beacon_file(dir_name + "beaconset_no_mac.csv");

	Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_set = beacon_file.extractDoulbeMatrix(",");

	double after_read_time = get_time();
	std::cout << "read file time:"
	          << after_read_time - start_read_time
	          << "[s]" << std::endl;

	auto uwb_tool = BSE::UwbTools(uwb_data,
	                              beacon_set);

	// calculate uwb trace
	Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();
	Eigen::Vector3d initial_pos = optimize_trace.block(0, 0, 1, 3).transpose();
	double initial_ori = uwb_tool.computeInitialOri(optimize_trace);


	double uwb_opt_time = get_time();
	std::cout << "cal uwb time:"
	          << uwb_opt_time - after_read_time
	          << "[s]" << std::endl;


	//pre-process imu data
	BSE::ImuTools::processImuData(imu_data);


	auto eskf_filter = IMUESKF(initial_pos, initial_ori);

	int imu_index = 0;
	int uwb_index = 0;
	while (imu_index < imu_data.rows() && uwb_index < uwb_data.rows()) {
		if (imu_data(imu_index, 0) < uwb_data(uwb_index, 0)) {
//			std::cout << imu_data.block(imu_index, 0,1, imu_data.cols()) << std::endl;

			imu_index += 1;
		} else {
			logger_ptr->addTrace3dEvent("Single",
			                            "UWB Optimize",
			                            optimize_trace.block(uwb_index,
			                                                 0, 1, 3));

			uwb_index += 1;
		}
	}

	logger_ptr->outputAllEvent(true);

}
