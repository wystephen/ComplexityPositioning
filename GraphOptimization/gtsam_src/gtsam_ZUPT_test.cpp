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

#include <gtsam/nonlinear/ISAM2.h>


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


PreintegrationType *imu_preintegrated_;

int main(int argc, char *argv[]) {

	std::cout.precision(30);
	// parameters
//    std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
//	std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
	std::string dir_name = "/home/steve/Data/ZUPTPDR/0001/";


	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	// load data
	AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
			right_foot_file(dir_name + "RIGHT_FOOT.data"),
			head_imu_file(dir_name + "HEAD.data");
//			uwb_file(dir_name + "uwb_result.csv"),
//			beacon_set_file(dir_name + "beaconSet.csv");

	Eigen::MatrixXd left_imu_data = left_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd right_imu_data = right_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd head_imu_data = head_imu_file.extractDoulbeMatrix(",");
//	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
//	Eigen::MatrixXd beacon_set_data = beacon_set_file.extractDoulbeMatrix(",");


	left_imu_data = right_imu_data * 1.0;

	assert(beacon_set_data.rows() == (uwb_data.cols() - 1));

	// get the initial pose based on uwb data.

//	std::cout << uwb_data.block(0, 0, 1, uwb_data.cols()) << std::endl;

//	auto uwb_tool = BSE::UwbTools(uwb_data,
//	                              beacon_set_data);

//	Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();
	Eigen::Vector3d initial_pos(0.0, 0.0, 0.0);// = optimize_trace.block(0, 0, 1, 3).transpose();
	double initial_ori = 0.0;//uwb_tool.computeInitialOri(optimize_trace);

	std::vector<std::vector<double>> optimize_trace_vec = {{},
	                                                       {},
	                                                       {}};
//	for (int i(0); i < optimize_trace.rows(); ++i) {
//		for (int j(0); j < 3; ++j) {
//			optimize_trace_vec[j].push_back(optimize_trace(i, j));
//		}
//	}


//    auto imu_tool = BSE::ImuTools();


	//process imu data
	BSE::ImuTools::processImuData(left_imu_data);
	BSE::ImuTools::processImuData(right_imu_data);
	BSE::ImuTools::processImuData(head_imu_data);
	double left_dt = (left_imu_data(left_imu_data.rows() - 1, 0) - left_imu_data(0, 0)) / double(left_imu_data.rows());


	Eigen::MatrixXd left_zv_state = Eigen::MatrixXd::Ones(left_imu_data.rows(), 1);
	Eigen::MatrixXd right_zv_state = Eigen::MatrixXd::Ones(right_imu_data.rows(), 1);
	Eigen::MatrixXd head_zv_state = Eigen::MatrixXd::Ones(head_imu_data.rows(), 1);

	auto zv_cal_function = [](Eigen::MatrixXd &zv_state, Eigen::MatrixXd &imu_data) {
		assert(zv_state.rows() == imu_data.rows());
		for (int i(6); i < imu_data.rows() - 6; ++i) {
			if (BSE::ImuTools::GLRT_Detector(imu_data.block(i - 5, 1, 10, 6))) {
				zv_state(i) = 1.0;
//				std::cout << "zv state" << std::endl;
			} else {
				zv_state(i) = 0.0;
//				std::cout << "non zv state" << std::endl;
			}
		}

	};
	zv_cal_function(left_zv_state, left_imu_data);
	zv_cal_function(right_zv_state, right_imu_data);
	zv_cal_function(head_zv_state, head_imu_data);



	// noise model
	noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
			(Vector(6) << 0.0001, 0.0001, 0.0001, 0.005, 0.005, 0.005).finished());
	noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.00001);
	noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-13);

	noiseModel::Diagonal::shared_ptr zero_velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-3);
//	noiseModel::Constrained::shared_ptr zero_velocity_noise_model =
//			noiseModel::Constrained::;


	// We use the sensor specs to build the noise model for the IMU factor.
	double accel_noise_sigma = 0.001;
	double gyro_noise_sigma = 0.001 * M_PI / 180.0;
	double accel_bias_rw_sigma = 0.0004905;
	double gyro_bias_rw_sigma = 0.00001454441043;
	Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
	Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
	Matrix33 integration_error_cov =
			Matrix33::Identity(3, 3) * 1e-8; // error committed in integrating position from velocities
	Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
	Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
	Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * 1e-5; // error in the bias used for preintegration

	boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p =
			PreintegratedCombinedMeasurements::Params::MakeSharedD(
					-9.8);
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

	auto tmp_q = BSE::ImuTools::initial_quaternion(left_imu_data.block(0, 1, 50, 3), initial_ori, false);

	Rot3 prior_rotation = Rot3::Quaternion(tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z());
	Point3 prior_point = initial_pos;
	Pose3 prior_pose(prior_rotation, prior_point);
	Vector3 prior_velocity(0.0, 0.0, 0.0);
	imuBias::ConstantBias prior_imu_bias; // assume zero initial bias.

	Values initial_values;
	NonlinearFactorGraph graph;
	int left_counter = 0;
	int left_normal_counter = 0;
	int left_zv_counter = 0;

	initial_values.insert(X(left_counter), prior_pose);
	initial_values.insert(V(left_counter), prior_velocity);
	initial_values.insert(B(left_counter), prior_imu_bias);

	graph.add(PriorFactor<Pose3>(X(left_counter), prior_pose, pose_noise_model));
//	graph.add()
	graph.add(PriorFactor<Vector3>(V(left_counter), prior_velocity, velocity_noise_model));
	graph.add(PriorFactor<imuBias::ConstantBias>(B(left_counter), prior_imu_bias, bias_noise_model));


	// initial isam2
	ISAM2Params parameters;
	parameters.relinearizeThreshold = 0.01;
	parameters.relinearizeSkip = 1;
	ISAM2 isam(parameters);

	isam.update(graph, initial_values);
	isam.update();

	graph.resize(0);
	initial_values.clear();


	imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);


	double last_rate = 0.0;
	for (int i(0); i < left_imu_data.rows() - 2; ++i) {
		Eigen::Vector3d acc(left_imu_data(i, 1), left_imu_data(i, 2), left_imu_data(i, 3));
		Eigen::Vector3d gyr(left_imu_data(i, 4), left_imu_data(i, 5), left_imu_data(i, 6));
//		imu_preintegrated_->integrateMeasurement(left_imu_data.block(i,1,1,3),
//				left_imu_data.block(i,4,1,3),left_dt);
		imu_preintegrated_->integrateMeasurement(acc, gyr, left_dt);

		double rate = double(i) / double(left_imu_data.rows());
		if (rate - last_rate > 0.05) {
			std::cout << "finished:" << rate * 100.0 << "%" << std::endl;
			last_rate = rate;
		}
		auto add_new_factor = [&](int &counter, double the_zv_flag) {

			counter += 1;
//			std::cout << "counter ;" << counter << std::endl;



//			auto prev_state = new NavState(prior_pose,prior_velocity);
//			imuBias::ConstantBias prev_bias = prior_imu_bias;
//			auto prop_state = imu_preintegrated_->predict(prev_state,prev_bias);
			initial_values.insert(X(counter), prior_pose);
			initial_values.insert(V(counter), prior_velocity);
			initial_values.insert(B(counter), prior_imu_bias);

			PreintegratedImuMeasurements *preint_imu =
					dynamic_cast<PreintegratedImuMeasurements *>(imu_preintegrated_);
			ImuFactor imu_factor(X(counter - 1), V(counter - 1),
			                     X(counter), V(counter),
			                     B(counter - 1), *preint_imu);
			graph.push_back(imu_factor);

			imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
			graph.push_back(BetweenFactor<imuBias::ConstantBias>(B(counter - 1),
			                                                     B(counter),
			                                                     zero_bias, bias_noise_model));

			if (the_zv_flag > 0.5) {
//				std::cout << "zv flag" << std::endl;
				graph.push_back(PriorFactor<Vector3>(
						V(counter), Eigen::Vector3d(0.0, 0.0, 0.0), zero_velocity_noise_model
				));


			}
			try {
				isam.update(graph, initial_values);
				isam.update();


				Values currentEstimate = isam.calculateEstimate();
//			currentEstimate.print("current state:");

				prior_pose = currentEstimate.at<Pose3>(X(counter));
				prior_velocity = currentEstimate.at<Vector3>(V(counter));
				prior_imu_bias = currentEstimate.at<imuBias::ConstantBias>(B(counter));

//				std::cout << prior_pose << std::endl;

				logger_ptr->addTrace3dEvent("trace", "real_time_gtsam",
				                            Eigen::Vector3d(prior_pose.x(), prior_pose.y(), prior_pose.z()));
				logger_ptr->addTraceEvent("trace", "real_time_gtsam",
				                          Eigen::Vector2d(prior_pose.x(), prior_pose.y()));

				logger_ptr->addPlotEvent("velocity", "velocity", prior_velocity);
			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
//				isam.
			}


			graph.resize(0);
			initial_values.clear();
			imu_preintegrated_->resetIntegrationAndSetBias(prior_imu_bias);

		};

		if (left_zv_state(i) > 0.5) {
			left_zv_counter++;
			if (left_zv_counter > 20 || (left_zv_state(i + 1) < 0.5)) {
				add_new_factor(left_counter, 1.0);
				left_zv_counter = 0;
//				std::cout << "zv hitted." << std::endl;
			}

		} else {
			left_normal_counter++;
			if (left_normal_counter > 100 || left_zv_state(i + 1) > 0.5) {
				add_new_factor(left_counter, 0.0);
				left_normal_counter = 0;
//				std::cout << "unzv hitted" << std::endl;

			}

		}
	}


	Values currentEstimate = isam.calculateEstimate();
	for (int i(0); i < left_counter; ++i) {

		prior_pose = currentEstimate.at<Pose3>(X(i));
		prior_velocity = currentEstimate.at<Vector3>(V(i));
		prior_imu_bias = currentEstimate.at<imuBias::ConstantBias>(B(i));

		std::cout << prior_pose << std::endl;

		logger_ptr->addTrace3dEvent("trace", "final_gtsam",
		                            Eigen::Vector3d(prior_pose.x(), prior_pose.y(), prior_pose.z()));
		logger_ptr->addTraceEvent("trace", "final_gtsam",
		                          Eigen::Vector2d(prior_pose.x(), prior_pose.y()));

		logger_ptr->addPlotEvent("velocity", "final_velocity", prior_velocity);
	}


	std::cout << "imu total time" << left_imu_data(left_imu_data.rows() - 1, 0) - left_imu_data(0, 0) << std::endl;

	logger_ptr->outputAllEvent(true);


}