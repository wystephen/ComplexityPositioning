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
// Created by steve on 18-3-8.
//



#include <iostream>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <BayesFilter/KalmanFilter/KFComplex.h>
#include <BayesFilter/KalmanFilter/KFComplexFull.h>
//#include <AWF.h>dd


#include "AWF.h"

#include "BayesFilter/KalmanFilter/IMUWBKFSimple.h"
#include "BayesFilter/KalmanFilter/IMUWBKFSimple.cpp"

#include "../AuxiliaryTool/UwbTools.h"
#include "../AuxiliaryTool/UwbTools.cpp"

#include "../AuxiliaryTool/ImuTools.h"
#include "../AuxiliaryTool/ImuTools.cpp"


namespace plt = matplotlibcpp;


int main(int argc, char *argv[]) {

	std::cout.precision(30);

	std::string dir_name = "/home/steve/Data/XsensUwb/MTI700/0004/";

	AWF::FileReader imu_file(dir_name + "imu.data");
	AWF::FileReader uwb_file(dir_name + "uwb_data.csv");
	AWF::FileReader beacon_file(dir_name + "beaconset_no_mac.csv");

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_data = beacon_file.extractDoulbeMatrix(",");
//    beacon_data.block(0, 1, beacon_data.rows(), 1) *= -1.0;

//    uwb_data.block(0, 0, uwb_data.rows(), 1) =
//            uwb_data.block(0, 0, uwb_data.rows(), 1) - double(uwb_data(0, 0) + imu_data(0, 0));
	double time_offset = double(uwb_data(0, 0) - imu_data(0, 0));
	std::cout << "time offset:"
	          << time_offset << std::endl;
	for (int i(0); i < uwb_data.rows(); ++i) {
//        uwb_data(i, 0) = uwb_data(i, 0) - time_offset;
		uwb_data(i, 0) = uwb_data(i, 0) - 8.0 * 60.0 * 60.0;//time_offset;
	}
	auto uwb_tool = BSE::UwbTools(uwb_data,
	                              beacon_data);

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


	Eigen::MatrixXd process_noise_matrix =
			Eigen::MatrixXd::Identity(6, 6);
	process_noise_matrix.block(0, 0, 3, 3) *= 0.1;
	process_noise_matrix.block(3, 3, 3, 3) *= (0.1 * M_PI / 180.0);


	Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
	initial_prob_matrix.block(0, 0, 3, 3) *= 0.001;
	initial_prob_matrix.block(3, 3, 3, 3) *= 0.001;
	initial_prob_matrix.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);

	Eigen::MatrixXd initial_prob_full_matrix = Eigen::MatrixXd::Identity(15, 15);
	initial_prob_full_matrix.block(0, 0, 9, 9) = initial_prob_matrix;
	initial_prob_full_matrix.block(9, 9, 3, 3) *= 0.001;
	initial_prob_full_matrix.block(12, 12, 3, 3) *= 0.001 * (M_PI / 180.0);


	auto filter = BSE::IMUWBKFSimple(initial_prob_matrix);
	filter.setTime_interval_(0.01);
	auto complex_filter = BSE::KFComplex(initial_prob_matrix);
	complex_filter.time_interval_ = 0.01;

	auto complex_full_filter = BSE::KFComplexFull(initial_prob_full_matrix);
	complex_full_filter.time_interval_ = 0.01;

	filter.initial_state(imu_data.block(0, 1, 10, 6),
	                     initial_ori + 10.0 / 180.0 * M_PI,
	                     initial_pos);


	complex_filter.initial_state(imu_data.block(0, 1, 10, 9),
	                             initial_ori + 10.0 / 180.0 * M_PI,
	                             initial_pos);


	complex_full_filter.initial_state(imu_data.block(0, 1, 10, 9),
	                                  initial_ori + 10.0 / 180.0 * M_PI,
	                                  initial_pos);

	filter.setLocal_g_(-9.884);
	complex_filter.local_g_ = -9.884;
	complex_full_filter.local_g_ = -9.884;

//    filter.IS_DEBUG = true;





	int uwb_index = 0;


	for (int i(0); i < imu_data.rows(); ++i) {
		filter.StateTransaction(imu_data.block(i, 1, 1, 6).transpose(),
		                        process_noise_matrix,
		                        BSE::StateTransactionMethodType::NormalRotation);
		complex_filter.StateTransIMU(imu_data.block(i, 1, 1, 6).transpose(),
		                             process_noise_matrix);


		auto state_T = filter.getTransformMatrix();
		Eigen::MatrixXd state_x = filter.getState_();
		Eigen::MatrixXd complex_x = complex_filter.state_x_;
//        std::cout << state_x.transpose().eval() << std::endl;
//        std::cout << state_x.transpose() << std::endl;
		if (uwb_data(uwb_index, 0) < imu_data(i, 0)) {

			Eigen::Matrix<double, 1, 1> measurement_noise_matrix;
			measurement_noise_matrix.resize(1, 1);
			measurement_noise_matrix(0, 0) = 0.1;

			logger_ptr->addPlotEvent("xsense_uwb", "uwb", uwb_data.block(uwb_index, 1, 1, uwb_data.cols() - 1));
			logger_ptr->addPlotEvent("xsense_uwb", "uwb_error", optimize_trace.block(uwb_index, 3, 1, 1));
			std::vector<Eigen::Vector4d> m_stack;
			std::vector<Eigen::Matrix<double, 1, 1>> cov_stack;
			for (int k(1); k < uwb_data.cols(); ++k) {
				if (uwb_data(uwb_index, k) < 0.0 ||
				    uwb_data(uwb_index, k) > 208.0 ||
				    optimize_trace(uwb_index, 3) > 1000.0) {
					break;
				} else {

					Eigen::Vector4d measurement_data(0, 0, 0, uwb_data(uwb_index, k));
					measurement_data.block(0, 0, 3, 1) = beacon_data.block(k - 1, 0, 1, 3).transpose();
//                std::cout << measurement_data.transpose() << std::endl;



					// Correcting state according to uwb measurement.
					filter.MeasurementState(measurement_data,
					                        measurement_noise_matrix * 0.00001,
					                        BSE::MeasurementMethodType::NormalUwbMeasuremnt);

//                    complex_filter.MeasurementUwb(measurement_data,
//                                                  measurement_noise_matrix * 0.001);
					m_stack.push_back(measurement_data);
					cov_stack.push_back(measurement_noise_matrix * 0.0001);
				}


			}

			Eigen::MatrixXd m_matrix(m_stack.size(), 4);
			Eigen::MatrixXd cov_matrix(cov_stack.size(), cov_stack.size());
			m_matrix.setZero();
			cov_matrix.setZero();
			for (int k(0); k < m_stack.size(); ++k) {
				m_matrix.block(k, 0, 1, 4) = m_stack[k].transpose();
				cov_matrix(k, k) = cov_stack[k](0);
			}
			complex_filter.MeasurementUwbFull(m_matrix, cov_matrix);

			uwb_index++;


		}

		auto filter_state = filter.getState_();
		auto complex_state = complex_filter.state_x_;

		logger_ptr->addTrace3dEvent("xsense_uwb", "filter_trace", filter_state.block(0, 0, 3, 1));
		logger_ptr->addTrace3dEvent("xsense_uwb", "complex_trace", complex_state.block(0, 0, 3, 1));
		logger_ptr->addTrace3dEvent("xsense_uwb", "uwb_optimize", optimize_trace.block(uwb_index, 0, 1, 3));

		logger_ptr->addPlotEvent("xsense_uwb_complex", "pos", complex_state.block(0, 0, 3, 1));
		logger_ptr->addPlotEvent("xsense_uwb_complex", "vel", complex_state.block(3, 0, 3, 1));
		logger_ptr->addPlotEvent("xsense_uwb_complex", "ang", complex_state.block(6, 0, 3, 1));


	}


	logger_ptr->outputAllEvent(true);


}
