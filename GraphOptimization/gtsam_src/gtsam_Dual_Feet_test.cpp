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
// Created by steve on 11/11/18.
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


#include <gtsam/sam/RangeFactor.h>


#include <gtsam_include/MaxDistanceConstraint.h>
#include <gtsam_include/DualFeetConstraint.hpp>

#include <gtsam_include/MaxRangeExpressionFactor.h>

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


PreintegrationType *imu_preintegrated_left_;
PreintegrationType *imu_preintegrated_right_;


int main(int argc, char *argv[]) {

	omp_set_num_threads(1);

	std::cout.precision(30);
	// parameters
//    std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
//	std::string dir_name = "/home/steve/Data/FusingLocationData/0012/";
	std::string dir_name = "/home/steve/Data/ZUPTPDR/0003/";


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


//	right_imu_data = left_imu_data * 1.0;

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
	double right_dt =
			(right_imu_data(left_imu_data.rows() - 1, 0) - right_imu_data(0, 0)) / double(right_imu_data.rows());


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
	double point_sigma = 1e-3;
	double theta_sigma = 1.0;
	noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
			(Vector(6) << point_sigma, point_sigma, point_sigma, theta_sigma, theta_sigma, theta_sigma).finished());
	noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.0001);
	noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-13);
	noiseModel::Diagonal::shared_ptr max_distance_model = noiseModel::Isotropic::Sigma(1, 1.0e-3);
//	noiseModel::Diagonal::shared_ptr max_distance_model = noiseModel::Constrained::All(1);

	noiseModel::Diagonal::shared_ptr zero_velocity_noise_model =
			noiseModel::Isotropic::Sigma(3, 1e-3);
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
	Rot3 prior_rotation_left = Rot3::Quaternion(tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z());
	Point3 prior_point_left = initial_pos;
	Pose3 prior_pose_left(prior_rotation_left, prior_point_left);
	Vector3 prior_velocity_left(0.0, 0.0, 0.0);
	imuBias::ConstantBias prior_imu_bias_left; // assume zero initial bias.


	tmp_q = BSE::ImuTools::initial_quaternion(right_imu_data.block(0, 1, 50, 3), initial_ori, false);
	Rot3 prior_rotation_right = Rot3::Quaternion(tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z());
	Point3 prior_point_right = initial_pos;
	Pose3 prior_pose_right(prior_rotation_right, prior_point_right);
	Vector3 prior_velocity_right(0.0, 0.0, 0.0);
	imuBias::ConstantBias prior_imu_bias_right; // assume zero initial bias.

	Values initial_values;
	NonlinearFactorGraph graph;
	int left_counter = 0;
	int left_normal_counter = 0;
	int left_zv_counter = 0;

	int right_counter = 0;
	int right_normal_counter = 0;
	int right_zv_counter = 0;

	long right_offset = 500000000;

	initial_values.insert(X(left_counter), prior_pose_left);
	initial_values.insert(V(left_counter), prior_velocity_left);
	initial_values.insert(B(left_counter), prior_imu_bias_left);

	initial_values.insert(X(right_offset + right_counter), prior_pose_right);
	initial_values.insert(V(right_offset + right_counter), prior_velocity_right);
	initial_values.insert(B(right_offset + right_counter), prior_imu_bias_right);

	graph.add(PriorFactor<Pose3>(X(left_counter), prior_pose_left, pose_noise_model));
	graph.add(PriorFactor<Vector3>(V(left_counter), prior_velocity_left, velocity_noise_model));
	graph.add(PriorFactor<imuBias::ConstantBias>(B(left_counter), prior_imu_bias_left, bias_noise_model));


	graph.add(PriorFactor<Pose3>(X(right_offset + right_counter), prior_pose_right, pose_noise_model));
	graph.add(PriorFactor<Vector3>(V(right_offset + right_counter), prior_velocity_right, velocity_noise_model));
	graph.add(PriorFactor<imuBias::ConstantBias>(B(right_offset + right_counter), prior_imu_bias_right,
	                                             bias_noise_model));


	// initial isam2
	ISAM2Params parameters;
	parameters.relinearizeThreshold = 0.001;
	parameters.relinearizeSkip = 1;
	ISAM2 isam(parameters);

	isam.update(graph, initial_values);
	isam.update();

	graph.resize(0);
	initial_values.clear();


	imu_preintegrated_left_ = new PreintegratedImuMeasurements(p, prior_imu_bias_left);
	imu_preintegrated_right_ = new PreintegratedImuMeasurements(p, prior_imu_bias_right);

	auto show_latest_state_func = [&]() {
		Values state_values = isam.calculateEstimate();
		std::cout << "last left:" << left_counter << "---------"
		          << state_values.at<Pose3>(X(left_counter)) << std::endl;
		std::cout << "last right:" << right_counter << "-=-----"
		          << state_values.at<Pose3>(X(right_counter + right_offset)) << std::endl;
		auto p1 = state_values.at<Pose3>(X(left_counter));
		auto p2 = state_values.at<Pose3>(X(right_counter + right_offset));
		std::cout << "distance between latest pose:"
		          << pow(pow(p1.x() - p2.x(), 2.0) + pow(p1.y() - p2.y(), 2.0) + pow(p1.z() - p2.z(), 2.0), 0.5)
		          << std::endl;

	};


	double last_rate = 0.0;
	for (int i(0); i < left_imu_data.rows() - 2 && i < right_imu_data.rows(); ++i) {
		Eigen::Vector3d acc_left(left_imu_data(i, 1),
		                         left_imu_data(i, 2),
		                         left_imu_data(i, 3));
		Eigen::Vector3d gyr_left(left_imu_data(i, 4),
		                         left_imu_data(i, 5),
		                         left_imu_data(i, 6));
		imu_preintegrated_left_->integrateMeasurement(acc_left, gyr_left,
		                                              left_dt);

		Eigen::Vector3d acc_right(right_imu_data(i, 1),
		                          right_imu_data(i, 2),
		                          right_imu_data(i, 3));
		Eigen::Vector3d gyr_right(right_imu_data(i, 4),
		                          right_imu_data(i, 5),
		                          right_imu_data(i, 6));
		imu_preintegrated_right_->integrateMeasurement(acc_right, gyr_right,
		                                               right_dt);//TODO: use right dt rather than left dt.

		double rate = double(i) / double(left_imu_data.rows());
		if (rate - last_rate > 0.05) {
			std::cout << "finished:" << rate * 100.0 << "%" << std::endl;
			last_rate = rate;
		}



		/**
		 * @brief offset=0 (left foot) else: offset = 1(right foot)
		 */
		auto add_new_factor_left = [&](int &counter,
		                               double the_zv_flag,
		                               int offset = 0) {

			counter += 1;

			initial_values.insert(X(counter), prior_pose_left);
			initial_values.insert(V(counter), prior_velocity_left);
			initial_values.insert(B(counter), prior_imu_bias_left);

			PreintegratedImuMeasurements *preint_imu =
					dynamic_cast<PreintegratedImuMeasurements *>(imu_preintegrated_left_);
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

			//
			try {

				// add graph and new values to isam and
				// update prior_pose_left prior velocity and prior bias.
				isam.update(graph, initial_values);
//				isam.update();
				Values currentEstimate = isam.calculateEstimate();

				if (counter % 100 == 0) {
					currentEstimate = isam.calculateBestEstimate();
				}

				//			currentEstimate.print("current state:");
				prior_pose_left = currentEstimate.at<Pose3>(X(counter));
				prior_velocity_left = currentEstimate.at<Vector3>(V(counter));
				prior_imu_bias_left = currentEstimate.at<imuBias::ConstantBias>(
						B(counter));
//				std::cout << prior_pose_left << std::endl;

				logger_ptr->addTrace3dEvent("trace", "real_time_gtsam",
				                            Eigen::Vector3d(prior_pose_left.x(),
				                                            prior_pose_left.y(),
				                                            prior_pose_left.z()));
				logger_ptr->addTraceEvent("trace", "real_time_gtsam",
				                          Eigen::Vector2d(prior_pose_left.x(),
				                                          prior_pose_left.y()));

				logger_ptr->addPlotEvent("velocity", "velocity", prior_velocity_left);
			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
				std::cout << "isam without the ability to recovery from ill-posed problem"
				          << std::endl;
				std::cout << "error at left part:" << left_counter
				          << "," << right_counter << std::endl;

				show_latest_state_func();
			}


			graph.resize(0);
			initial_values.clear();
			imu_preintegrated_left_->resetIntegrationAndSetBias(prior_imu_bias_left);

		};

		if (left_zv_state(i) > 0.5) {
			left_zv_counter++;
			if (left_zv_counter > 20 || (left_zv_state(i + 1) < 0.5)) {
				add_new_factor_left(left_counter, 1.0);
				left_zv_counter = 0;
//				std::cout << "zv hitted." << std::endl;
			}

		} else {
			left_normal_counter++;
			if (left_normal_counter > 100 || left_zv_state(i + 1) > 0.5) {
				add_new_factor_left(left_counter, 0.0);
				left_normal_counter = 0;
//			std::cout << "unzv hitted" << std::endl;

			}

		}





		/**
		* @brief offset=0 (left foot) else: offset = 1(right foot)
		*/
		auto add_new_factor_right = [&](int &counter,
		                                const double the_zv_flag,
		                                int offset = 0) {

			counter += 1;


			initial_values.insert(X(counter + right_offset), prior_pose_right);
			initial_values.insert(V(counter + right_offset), prior_velocity_right);
			initial_values.insert(B(counter + right_offset), prior_imu_bias_right);

			PreintegratedImuMeasurements *preint_imu =
					dynamic_cast<PreintegratedImuMeasurements *>(imu_preintegrated_right_);
			ImuFactor imu_factor(X(counter - 1 + right_offset), V(counter - 1 + right_offset),
			                     X(counter + right_offset), V(counter + right_offset),
			                     B(counter - 1 + right_offset), *preint_imu);
			graph.push_back(imu_factor);

			imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
			graph.push_back(BetweenFactor<imuBias::ConstantBias>(B(counter - 1 + right_offset),
			                                                     B(counter + right_offset),
			                                                     zero_bias, bias_noise_model));

			if (the_zv_flag > 0.5) {
//				std::cout << "zv flag" << std::endl;
				graph.push_back(PriorFactor<Vector3>(
						V(counter + right_offset),
						Eigen::Vector3d(0.0, 0.0, 0.0),
						zero_velocity_noise_model
				));
			}

			// Add max distance constraint
			if (true && right_counter > 1 && left_counter > 1 && the_zv_flag > 0.5) {


				//  Deleted Max distance constraint
//				MaxDistanceConstraint max_factor(X(left_counter ),
//				                                          X(counter + right_offset),
//				                                          1.5, false);
//				graph.push_back(max_factor);


//				graph.push_back(DualFeetConstraint(X(left_counter -1),
//				                                   X(right_counter + right_offset-1),
//				                                   1.0));
//			graph.push_back(DualFeetConstraint(X(1),X(1+right_offset),1.0));


				graph.push_back(MaxRangeExpressionFactor(X(left_counter),
				                                         X(right_counter + right_offset),
				                                         0.0, max_distance_model));

//				graph.push_back(RangeFactor<Pose3>(X(left_counter),
//						X(right_counter+right_offset),0.0,max_distance_model));


			}


			//
			try {

				// add graph and new values to isam and update prior_pose_left prior velocity and prior bias.
				std::cout << "left:" << left_counter << "  right:" << right_counter << std::endl;
				isam.update(graph, initial_values);
//				isam.update();
				Values currentEstimate = isam.calculateEstimate();
				//			currentEstimate.print("current state:");
				prior_pose_right = currentEstimate.at<Pose3>(X(counter + right_offset));
				prior_velocity_right = currentEstimate.at<Vector3>(V(counter + right_offset));
				prior_imu_bias_right = currentEstimate.at<imuBias::ConstantBias>(
						B(counter + right_offset));

				logger_ptr->addTrace3dEvent("trace", "real_time_gtsam_right",
				                            Eigen::Vector3d(prior_pose_right.x(), prior_pose_right.y(),
				                                            prior_pose_right.z()));
				logger_ptr->addTraceEvent("trace", "real_time_gtsam_right",
				                          Eigen::Vector2d(prior_pose_right.x(), prior_pose_right.y()));

				logger_ptr->addPlotEvent("velocity", "velocity_right", prior_velocity_right);
			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
				std::cout << "isam without the ability to recovery from ill-posed problem" << std::endl;

				std::cout << "error at right part:" << left_counter << "," << right_counter << std::endl;
				show_latest_state_func();
			}


			graph.resize(0);
			initial_values.clear();
			imu_preintegrated_right_->resetIntegrationAndSetBias(prior_imu_bias_right);

		};


		if (right_zv_state(i) > 0.5) {
			right_zv_counter++;
			if (right_zv_counter > 20 || right_zv_state(i + 1) < 0.5) {
				add_new_factor_right(right_counter, 1.0);
//				add_new_factor_left(right_counter, 1.0);
				right_zv_counter = 0;
			}

		} else {
			right_normal_counter++;
			if (right_normal_counter > 100 || right_zv_state(i + 1) > 0.5) {
				add_new_factor_right(right_counter, 0.0);
//				add_new_factor_left(right_counter, 0.0);
				right_normal_counter = 0;
			}

		}

	}


	// Visualize final result. (both left and right foot)
	Values currentEstimate = isam.calculateEstimate();
	for (int i(0); i < left_counter; ++i) {
		try {
			prior_pose_left = currentEstimate.at<Pose3>(X(i));
			prior_velocity_left = currentEstimate.at<Vector3>(V(i));
			prior_imu_bias_left = currentEstimate.at<imuBias::ConstantBias>(B(i));

//		std::cout << prior_pose_left << std::endl;

			logger_ptr->addTrace3dEvent("trace", "final_gtsam",
			                            Eigen::Vector3d(prior_pose_left.x(),
			                                            prior_pose_left.y(),
			                                            prior_pose_left.z()

			                            ));
			logger_ptr->addTraceEvent("trace", "final_gtsam",
			                          Eigen::Vector2d(prior_pose_left.x(),
			                                          prior_pose_left.y()

			                          ));

			logger_ptr->addPlotEvent("velocity", "final_velocity", prior_velocity_left);

		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;
			break;
		}


	}
	for (int i(right_offset); i < right_counter + right_offset; ++i) {
		try {
			prior_pose_right = currentEstimate.at<Pose3>(X(i));
			prior_velocity_right = currentEstimate.at<Vector3>(V(i));
			prior_imu_bias_right = currentEstimate.at<imuBias::ConstantBias>(B(i));

			logger_ptr->addTrace3dEvent("trace", "final_gtsam_right",
			                            Eigen::Vector3d(prior_pose_right.x(),
			                                            prior_pose_right.y(),
			                                            prior_pose_right.z()

			                            ));
			logger_ptr->addTraceEvent("trace", "final_gtsam_right",
			                          Eigen::Vector2d(prior_pose_right.x(),
			                                          prior_pose_right.y()

			                          ));
			logger_ptr->addPlotEvent("velocity", "final_gtsam_velocity_right", prior_velocity_right);

		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;

			break;
		}

	}


	std::cout << "imu total time" <<
	          left_imu_data(left_imu_data.rows() - 1, 0) - left_imu_data(0, 0) <<
	          std::endl;


	logger_ptr->outputAllEvent(true);


}