/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 10/29/18.
//


#include <iostream>
#include <fstream>


#include <thread>
#include <AWF.h>


#include "AWF.h"


#include "BSE.h"

#include <Eigen/Eigen>


// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

#include <random>
// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
//#define USE_COMBINED

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

using symbol_shorthand::Z; // zero velocity


namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {

	std::cout.precision(30);
	// parameters
//    std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
	std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";


	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	// load data
	AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
			right_foot_file(dir_name + "RIGHT_FOOT.data"),
			head_imu_file(dir_name + "HEAD.data"),
			uwb_file(dir_name + "uwb_result.csv"),
			beacon_set_file(dir_name + "beaconSet.csv");

	Eigen::MatrixXd left_imu_data = left_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd right_imu_data = right_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd head_imu_data = head_imu_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_set_data = beacon_set_file.extractDoulbeMatrix(",");

	assert(beacon_set_data.rows() == (uwb_data.cols() - 1));

	// get the initial pose based on uwb data.

	std::cout << uwb_data.block(0, 0, 1, uwb_data.cols()) << std::endl;

	auto uwb_tool = BSE::UwbTools(uwb_data,
	                              beacon_set_data);

	Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();
	Eigen::Vector3d initial_pos = optimize_trace.block(0, 0, 1, 3).transpose();
	double initial_ori = uwb_tool.computeInitialOri(optimize_trace);

	std::vector<std::vector<double>> optimize_trace_vec = {{},
	                                                       {},
	                                                       {}};
	for (int i(0); i < optimize_trace.rows(); ++i) {
		for (int j(0); j < 3; ++j) {
			optimize_trace_vec[j].push_back(optimize_trace(i, j));
		}
	}


//    auto imu_tool = BSE::ImuTools();


	//process imu data
	BSE::ImuTools::processImuData(left_imu_data);
	BSE::ImuTools::processImuData(right_imu_data);
	BSE::ImuTools::processImuData(head_imu_data);


	// noise model
	noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
			(Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
	noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);
	noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

	noiseModel::Diagonal::shared_ptr zero_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-3);

	// We use the sensor specs to build the noise model for the IMU factor.
	double accel_noise_sigma = 0.0003924;
	double gyro_noise_sigma = 0.000205689024915;
	double accel_bias_rw_sigma = 0.004905;
	double gyro_bias_rw_sigma = 0.000001454441043;
	Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
	Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
	Matrix33 integration_error_cov =
			Matrix33::Identity(3, 3) * 1e-8; // error committed in integrating position from velocities
	Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
	Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
	Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * 1e-5; // error in the bias used for preintegration

	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(
			0.0);
	// PreintegrationBase params:
	p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
	p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
	// should be using 2nd order integration
	// PreintegratedRotation params:
	p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
	// PreintegrationCombinedMeasurements params:
	p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
	p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
	p->biasAccOmegaInt = bias_acc_omega_int;


	//Initial state

	auto tmp_q = BSE::ImuTools::initial_quaternion(left_imu_data.block(0,1,50,3),0.0,true);

	Rot3 prior_rotation  = Rot3::Quaternion(1.0,0.0,0.0,0.0);


}