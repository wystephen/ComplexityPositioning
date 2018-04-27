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
// Created by steve on 18-4-27.
//

#include <iostream>
#include <thread>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <AWF.h>

#include <BayesFilter/KalmanFilter/UKFComplexCraft.h>

#include <AuxiliaryTool/UwbTools.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"


#include <OwnEdge/DistanceEdge.h>
#include <OwnEdge/DistanceEdge.cpp>

#include <OwnEdge/SimpleDistanceEdge.h>
 #include <OwnEdge/SimpleRobustDistanceEdge.h>

int main(int argc, char *argv[]){
	omp_set_num_threads(12);

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
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

//    std::cout << uwb_data.block(0, 0, 1, uwb_data.cols()) << std::endl;

	auto uwb_tool = BSE::UwbTools(uwb_data,
	                              beacon_set_data);

	Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();
	Eigen::Vector3d initial_pos = optimize_trace.block(0, 0, 1, 3).transpose();
	double initial_ori = uwb_tool.computeInitialOri(optimize_trace);

	//process
	BSE::ImuTools::processImuData(left_imu_data);
	BSE::ImuTools::processImuData(right_imu_data);
	BSE::ImuTools::processImuData(head_imu_data);


	Eigen::MatrixXd initial_prob_matrix_complex = Eigen::MatrixXd::Identity(15, 15);
	initial_prob_matrix_complex.block(0, 0, 3, 3) *= 0.001;
	initial_prob_matrix_complex.block(3, 3, 3, 3) *= 0.001;
	initial_prob_matrix_complex.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);
	initial_prob_matrix_complex.block(9, 9, 3, 3) *= 0.0001;
	initial_prob_matrix_complex.block(12, 12, 3, 3) *= 0.0001 * (M_PI / 180.0);

	auto left_filter = BSE::UKFComplexCraft(initial_prob_matrix_complex);
	auto right_filter = BSE::UKFComplexCraft(initial_prob_matrix_complex);

	double time_interval = (left_imu_data(left_imu_data.rows() - 1, 0) - left_imu_data(0, 0))
	                           / double(left_imu_data.rows());

	left_filter.time_interval_ = time_interval;
	right_filter.time_interval_ = time_interval;

	left_filter.local_g_ = -9.81;
	right_filter.local_g_ = -9.81;

	for (int i(5); i < left_imu_data.rows() - 5; ++i) {
		/// state transaction equation
//		filter.StateTransaction(imu_data.block(i, 1, 1, 6).transpose(),
//		                        process_noise_matrix,
//		                        BSE::StateTransactionMethodType::NormalRotation);
		double uwb_index = 0;
		/// uwb measurement
		bool tmp_break_flag = false;
//		while (uwb_data(uwb_index, 0) < imu_data(i, 0)) {
//			uwb_index++;
//			if (uwb_index == uwb_data.rows()) {
//				uwb_index--;
//				break;
//
//			}
//		}
//		if (uwb_data(uwb_index, 0) - imu_data(i, 0) < 0.01) {
//
//			for (int k(1); k < uwb_data.cols(); ++k) {
//				if (uwb_data(uwb_index, k) > 0
//				    && uwb_data(uwb_index, k) < 100.0
//				    && optimize_trace(uwb_index, 3) < 2.0) {
//
//					Eigen::Vector4d measurement_data(0, 0, 0, uwb_data(uwb_index, k));
//					measurement_data.block(0, 0, 3, 1) = beacon_set_data.block(k - 1, 0, 1, 3).transpose();
//					measurement_noise_matrix.resize(1, 1);
//					if (uwb_index < 15) {
//
//						measurement_noise_matrix(0, 0) = 0.01;
//					} else {
//						measurement_noise_matrix(0, 0) = 0.1;
//					}
//					 correct
//                        filter.MeasurementState(measurement_data,
//                                                measurement_noise_matrix,
//                                                BSE::MeasurementMethodType::NormalUwbMeasuremnt);

//				}
//			}
//		}


		if (BSE::ImuTools::GLRT_Detector(left_imu_data.block(i - 5, 1, 10, 6))) {
			/// zero velocity detector
//			filter.MeasurementState(Eigen::Vector3d(0, 0, 0),
//			                        Eigen::Matrix3d::Identity() * 0.000251001,
//			                        BSE::MeasurementMethodType::NormalZeroVeclotiMeasurement);

			/// angle constraint through acc.
//                filter.MeasurementState(imu_data.block(i, 1, 1, 3).transpose(),
//                                        Eigen::Matrix3d::Identity() * 0.0001,
//                                        BSE::MeasurementMethodType::NormalAngleConstraint);


			}

		} else {
		}



	}



}

