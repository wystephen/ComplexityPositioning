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

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"


#include <OwnEdge/DistanceEdge.h>
#include <OwnEdge/SimpleRobustDistanceEdge.h>
#include <OwnEdge/SimpleDistanceEdge.h>
#include <OwnEdge/SimpleDistanceEdge.cpp>
#include <OwnEdge/PesudoRansacDistance.h>
#include <OwnEdge/HardConstraintIMU.h>
#include <OwnEdge/ZoEdge.h>
#include <OwnEdge/ZoEdge.cpp>

int main(int argc, char *argv[]) {
	omp_set_num_threads(12);

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

//	std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";
//	// load data
//	AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
//			right_foot_file(dir_name + "RIGHT_FOOT.data"),
//			head_imu_file(dir_name + "HEAD.data"),
//			uwb_file(dir_name + "uwb_result.csv"),
//			beacon_set_file(dir_name + "beaconSet.csv");
	std::string dir_name = "/home/steve/Data/NewFusingLocationData/0032/";
	// load data
	AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
			right_foot_file(dir_name + "RIGHT_FOOT.data"),
			head_imu_file(dir_name + "HEAD.data"),
			uwb_file(dir_name + "uwb_data.csv"),
			beacon_set_file(dir_name + "beaconset_no_mac.csv"),
			select_trace_file(dir_name + "selected_uwb_trace.csv");


	Eigen::MatrixXd left_imu_data = left_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd right_imu_data = right_foot_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd head_imu_data = head_imu_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_set_data = beacon_set_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd select_trace = select_trace_file.extractDoulbeMatrix(",");

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

	Eigen::MatrixXd process_noise_matrix =
			Eigen::MatrixXd::Identity(6, 6);
	process_noise_matrix.block(0, 0, 3, 3) *= 0.1;
	process_noise_matrix.block(3, 3, 3, 3) *= (0.1 * M_PI / 180.0);

	Eigen::MatrixXd measurement_noise_matrix =
			Eigen::MatrixXd::Identity(uwb_data.cols() - 1, uwb_data.cols() - 1);
	measurement_noise_matrix *= 0.1;


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

	left_filter.initial_state(left_imu_data.block(0, 1, 50, 6), initial_ori, initial_pos);
	right_filter.initial_state(right_imu_data.block(0, 1, 50, 6), initial_ori, initial_pos);

	int uwb_index = 0.0;


	/**
	 * Initial graph
	 */
	g2o::SparseOptimizer globalOptimizer;


//    // Initial solver


	// create the linear solver
	auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

	// create the block solver on top of the linear solver
	auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

	// create the algorithm to carry out the optimization
	g2o::OptimizationAlgorithmLevenberg *optimizationAlgorithm =
			new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));


	globalOptimizer.setAlgorithm(optimizationAlgorithm);




	// set left vertex index and right vertex index
	int left_vertex_index_init = 1000000;
	int right_vertex_index_init = 2000000;
	int uwb_vertex_index_init = 3000000;
	int left_vertex_index = left_vertex_index_init;
	int right_vertex_index = right_vertex_index_init;
	int uwb_vertex_index = uwb_vertex_index_init;

	auto last_left_transform = BSE::ImuTools::build_transform_matrix<double>(left_filter.state_x_.block(0, 0, 3, 1),
	                                                                         left_filter.rotation_q_);
	auto last_right_transform = BSE::ImuTools::build_transform_matrix<double>(right_filter.state_x_.block(0, 0, 3, 1),
	                                                                          right_filter.rotation_q_);

	auto *left_first_vertex = new g2o::VertexSE3();
	left_first_vertex->setId(left_vertex_index);
	left_vertex_index++;
	globalOptimizer.addVertex(left_first_vertex);

	auto *right_first_vertex = new g2o::VertexSE3();
	right_first_vertex->setId(right_vertex_index);
	right_vertex_index++;
	globalOptimizer.addVertex(right_first_vertex);



	/**graph parameter**/
	/// g2o parameter

	double first_info = 10000.0;
	double second_info = 10000.0;


	double distance_info = 0.001;
	double distance_sigma = 2.0;


	double z_offset = 1.90 - 1.12;

	double turn_threshold = 1.0;
	double corner_ratio = 10.0;

	int max_optimize_times = 4000;

	double time_offset = 0.0;


	double uwb_err_threshold = 0.5;

	int delay_times = 25;

	int out_delay_times = 4;

	int data_num = 5;

	std::vector<PesudoRansacDistance *> dis_edge_stack;

	std::vector<double> left_vector_time;
	left_vector_time.push_back(left_imu_data(0, 0));


	/**
	 * MAIN LOOP.!!
	 */
	for (int i(5); i < left_imu_data.rows() - 17 && i < right_imu_data.rows() - 17; ++i) {
		/// state transaction equation
		left_filter.StateTransIMU_jac(left_imu_data.block(i, 1, 1, 6).transpose(),
		                              process_noise_matrix
		);
		right_filter.StateTransIMU_jac(right_imu_data.block(i, 1, 1, 6).transpose(),
		                               process_noise_matrix
		);





		/// uwb measurement
		bool tmp_break_flag = false;
		if (uwb_index < select_trace.rows() && select_trace(uwb_index, 0) < left_imu_data(i, 0)) {


			auto sv = new g2o::VertexSE3();
			double *data = new double[6];
			for (int k(0); k < 3; ++k) {
				data[k] = select_trace(uwb_index, k + 1);
			}

			sv->setId(uwb_vertex_index);
			uwb_vertex_index++;
			sv->setEstimateData(data);
			sv->setFixed(true);
			globalOptimizer.addVertex(sv);


			Eigen::Matrix<double, 1, 1> info_matrix;
			info_matrix(0, 0) = distance_info;

			auto *left_dis_edge = new PesudoRansacDistance();
			left_dis_edge->vertices()[0] = globalOptimizer.vertex(uwb_vertex_index - 1);
			left_dis_edge->vertices()[1] = globalOptimizer.vertex(left_vertex_index - 1);

			left_dis_edge->setMeasurement(0.0);
			left_dis_edge->setInformation(info_matrix);

			dis_edge_stack.push_back(left_dis_edge);

			globalOptimizer.addEdge(left_dis_edge);

			auto *right_dis_edge = new DistanceEdge();
			right_dis_edge->vertices()[0] = globalOptimizer.vertex(uwb_vertex_index - 1);
			right_dis_edge->vertices()[1] = globalOptimizer.vertex(right_vertex_index - 1);

			right_dis_edge->setMeasurement(0.0);
			right_dis_edge->setInformation(info_matrix);
			globalOptimizer.addEdge(right_dis_edge);


			uwb_index++;


		}

		// IMU Transaction

		if (BSE::ImuTools::GLRT_Detector(left_imu_data.block(i - 5, 1, 10, 6))) {
			/// zero velocity detector
			left_filter.MeasurementStateZV(Eigen::Matrix3d::Identity() * 0.001);

			if (!BSE::ImuTools::GLRT_Detector(left_imu_data.block(i - 4, 1, 10, 6))) {
				// add left foot vertex
				auto *vertex_imu = new g2o::VertexSE3();
				vertex_imu->setId(left_vertex_index);
				left_vertex_index++;//
				globalOptimizer.addVertex(vertex_imu);

				left_vector_time.push_back(left_imu_data(i, 0));

				auto *e = new g2o::EdgeSE3();

				e->vertices()[0] = globalOptimizer.vertex(left_vertex_index - 2);
				e->vertices()[1] = globalOptimizer.vertex(left_vertex_index - 1);

				Eigen::Matrix<double, 6, 6> information(Eigen::Matrix<double, 6, 6>::Identity());

				information(0, 0) = information(1, 1) = information(2, 2) = first_info;
				information(3, 3) = information(4, 4) = information(5, 5) = second_info;

//            if (is_corner) {
//                information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
//                information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
//            }
				Eigen::Isometry3d tmp_transform = BSE::ImuTools::build_transform_matrix<double>(
						left_filter.state_x_.block(0, 0, 3, 1),
						left_filter.rotation_q_);

				e->setInformation(information);
				e->setMeasurement(last_left_transform.inverse() * tmp_transform);
				globalOptimizer.addEdge(e);


				auto *ce = new HardConstraintIMU();
				ce->vertices()[0] = globalOptimizer.vertex(left_vertex_index - 2);
				ce->vertices()[1] = globalOptimizer.vertex(left_vertex_index - 1);

				Eigen::Matrix<double, 1, 1> info;
				info(0, 0) = 1.0;
				ce->setInformation(info);
				ce->setMeasurement(last_left_transform.inverse() * tmp_transform);
//				globalOptimizer.addEdge(ce);

				last_left_transform = tmp_transform;

				auto* ze = new Z0Edge();
				Eigen::Matrix<double,1,1> info_z;
				info_z(0,0)=2.0;
				ze->setMeasurement(select_trace.block(0,3,select_trace.rows(),1).mean());
				ze->setInformation(info_z);
				ze->vertices()[0] = globalOptimizer.vertex(left_vertex_index-2);
				ze->vertices()[1] = globalOptimizer.vertex(left_vertex_index-1);

				globalOptimizer.addEdge(ze);

			}

		}
		if (BSE::ImuTools::GLRT_Detector(right_imu_data.block(i - 5, 1, 10, 6))) {
			/// zero velocity detector
			right_filter.MeasurementStateZV(Eigen::Matrix3d::Identity() * 0.001);
			if (!BSE::ImuTools::GLRT_Detector(right_imu_data.block(i - 4, 1, 10, 6))) {
				// add right foot vertex
				auto *vertex_imu = new g2o::VertexSE3();
				vertex_imu->setId(right_vertex_index);
				right_vertex_index++;
				globalOptimizer.addVertex(vertex_imu);

				auto *e = new g2o::EdgeSE3();

				e->vertices()[0] = globalOptimizer.vertex(right_vertex_index - 2);
				e->vertices()[1] = globalOptimizer.vertex(right_vertex_index - 1);

				Eigen::Matrix<double, 6, 6> information(Eigen::Matrix<double, 6, 6>::Identity());

				information(0, 0) = information(1, 1) = information(2, 2) = first_info;
				information(3, 3) = information(4, 4) = information(5, 5) = second_info;

//            if (is_corner) {
//                information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
//                information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
//            }
				Eigen::Isometry3d tmp_transform = BSE::ImuTools::build_transform_matrix<double>(
						right_filter.state_x_.block(0, 0, 3, 1),
						right_filter.rotation_q_);

				e->setInformation(information);
				e->setMeasurement(last_right_transform.inverse() * tmp_transform);
				globalOptimizer.addEdge(e);

				last_right_transform = tmp_transform;

			}

		}
//		logger_ptr->addTraceEvent("trace", "left", left_filter.state_x_.block(0, 0, 2, 1));
//		logger_ptr->addTraceEvent("trace", "right", right_filter.state_x_.block(0, 0, 2, 1));
//		logger_ptr->addTrace3dEvent("trace", "left", left_filter.state_x_.block(0, 0, 3, 1));
//		logger_ptr->addTrace3dEvent("trace", "right", right_filter.state_x_.block(0, 0, 3, 1));
//		logger_ptr->addTraceEvent("traceimu", "left", left_filter.state_x_.block(0, 0, 2, 1));
//		logger_ptr->addTraceEvent("traceimu", "right", right_filter.state_x_.block(0, 0, 2, 1));
//		logger_ptr->addTrace3dEvent("traceimu", "left", left_filter.state_x_.block(0, 0, 3, 1));
//		logger_ptr->addTrace3dEvent("traceimu", "right", right_filter.state_x_.block(0, 0, 3, 1));


	}

	globalOptimizer.initializeOptimization();
	globalOptimizer.setVerbose(true);

	globalOptimizer.optimize(15000);


	double *data_ptr = new double[10];

	std::ofstream out_ref_trace(dir_name + "ref_trace.csv");// out put ref trace based on uwb and foot-mounted imu.
	out_ref_trace.precision(15);
	for (int i(left_vertex_index_init); i < left_vertex_index; ++i) {
		globalOptimizer.vertex(i)[0].getEstimateData(data_ptr);
		logger_ptr->addTrace3dEvent("trace", "left_graph", Eigen::Vector3d(data_ptr[0], data_ptr[1], data_ptr[2]));
		logger_ptr->addTraceEvent("trace", "left_graph", Eigen::Vector3d(data_ptr[0], data_ptr[1], data_ptr[2]));
//		std::cout << "left:"<<i <<  ":" << data_ptr[0] << "," << data_ptr[1] << "," << data_ptr[2] << std::endl;
		out_ref_trace << left_vector_time[i - left_vertex_index_init]
		              << ","
		              << data_ptr[0]
		              << ","
		              << data_ptr[1]
		              << ","
		              << data_ptr[2]
		              << "\n";

	}
	out_ref_trace.close();

	for (int i(right_vertex_index_init); i < right_vertex_index; ++i) {
		globalOptimizer.vertex(i)[0].getEstimateData(data_ptr);
		logger_ptr->addTrace3dEvent("trace", "right_graph", Eigen::Vector3d(data_ptr[0], data_ptr[1], data_ptr[2]));
		logger_ptr->addTraceEvent("trace", "right_graph", Eigen::Vector3d(data_ptr[0], data_ptr[1], data_ptr[2]));

	}

	for (int i(0); i < optimize_trace.rows(); ++i) {

		logger_ptr->addTrace3dEvent("trace", "uwb_optimiz"
		                                     ""
		                                     "e", optimize_trace.block(i, 0, 1, 3));
		logger_ptr->addTraceEvent("trace", "uwb_optimize", optimize_trace.block(i, 0, 1, 3));
	}

	logger_ptr->outputAllEvent(true);


}

