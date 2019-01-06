//
// Created by steve on 1/5/19.
//



#include <iostream>
#include <fstream>


#include <thread>
#include <AWF.h>


#include "AWF.h"


#include "BSE.h"

#include <Eigen/Eigen>


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/slam/RangeFactor.h>


#include <gtsam_include/UwbFactor.h>

using namespace gtsam;
using namespace std;

using symbol_shorthand::X;// pose2(x,y,theta)

using symbol_shorthand::L;//landmark (uwb beacon)


namespace plt=matplotlibcpp;

int main() {
	std::string dir_name = "/home/steve/Data/PDRUWBRobust/";

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();

	AWF::FileReader beacon_set_file(dir_name + "beacon_coordinate.csv"),
			pdr_file(dir_name + "pdr_result.csv"),
			uwb_file(dir_name + "uwb_noise.csv");

	Eigen::MatrixXd pdr_data = pdr_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_set = beacon_set_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");


	// graph and values
	NonlinearFactorGraph graph;
	Values values;

	NonlinearFactorGraph robust_graph;
	Values robust_values;

	int pose_counter = 0;


	// define noise model.
	auto pose2_prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1 * M_PI));
//	auto beacon_set_noise = noiseModel::Constrained::All(2);
	auto beacon_set_noise = noiseModel::Constrained::Sigmas(Vector2(0.1, 0.1));
	auto odometry_noise = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.21));
	auto base_range_noise = noiseModel::Diagonal::Sigmas((Vector(1) << 0.5).finished());
	auto range_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Tukey::Create(1.0),
	                                              base_range_noise);  //noiseModel::Robust(noise::noiseModel::Isotropic::Sigma(1,0.1));


	// add prior constraint
	Pose2 initial_pose = Pose2(0.15, 5.0, 0.0);
	Pose2 robust_initial_pose = Pose2(0.15, 5.0, 0.0);
	values.insert(X(pose_counter), initial_pose);
	robust_values.insert(X(pose_counter), robust_initial_pose);

	graph.add(
			PriorFactor<Pose2>(X(pose_counter), initial_pose, pose2_prior_noise)
	);
	robust_graph.add(
			PriorFactor<Pose2>(X(pose_counter), robust_initial_pose, pose2_prior_noise)
	);

	for (int i = 0; i < beacon_set.rows(); ++i) {
		Point2 beacon_set_point = Point2(beacon_set(i, 0), beacon_set(i, 1));
		values.insert(L(i), beacon_set_point);
//		robust_values.insert(L(i), beacon_set_point);
		graph.add(PriorFactor<Point2>(L(i), beacon_set_point, beacon_set_noise));
//		robust_graph.add(PriorFactor<Point2>(L(i), beacon_set_point, beacon_set_noise));
	}

	//initial isam2
	ISAM2Params params;
	ISAM2 isam2(params);

	isam2.update(graph, values);
	isam2.update();

	graph.resize(0);
	values.clear();

	ISAM2Params robust_params;
	ISAM2 robust_isam2(robust_params);

	robust_isam2.update(robust_graph, robust_values);
	robust_isam2.update();

	robust_graph.resize(0);
	robust_values.clear();

	int uwb_index = 0;

	// positioning
	for (int i = 0; i < pdr_data.rows(); ++i) {
		pose_counter++;
		//odometry constraint
		values.insert(X(pose_counter), initial_pose);
		graph.add(
				BetweenFactor<Pose2>(
						X(pose_counter - 1), X(pose_counter),
						Pose2(pdr_data(i, 1), 0.0, pdr_data(i, 3)),
						odometry_noise
				)
		);

		robust_values.insert(X(pose_counter), robust_initial_pose);
		robust_graph.add(
				BetweenFactor<Pose2>(
						X(pose_counter - 1), X(pose_counter),
						Pose2(pdr_data(i, 1), 0.0, pdr_data(i, 3)),
						odometry_noise
				)
		);
		while (uwb_data(uwb_index, 0) < pdr_data(i, 0)) {

			std::vector<double> valid_range;
			std::vector<Vector2> valid_beacon_set;
			for (int k = 0; k < beacon_set.rows(); ++k) {
				if (uwb_data(uwb_index, k + 1) > 0.0) {
					//insert uwb range constraint
					graph.add(
							RangeFactor<Pose2, Point2>(
									X(pose_counter),
									L(k),
									uwb_data(uwb_index, k + 1),
									range_noise
							)
					);
					valid_range.push_back(uwb_data(uwb_index, k + 1));
					valid_beacon_set.push_back(Vector2(beacon_set(k, 0), beacon_set(k, 1)));

					//TODO: ROUBST VERSION SHOULD BE ADOPTED.
//					robust_graph.add(
//							RangeFactor<Pose2, Point2>(
//									X(pose_counter),
//									L(k),
//									uwb_data(uwb_index, k + 1),
//									range_noise
//							)
//					);
//					printf("beacon:%d Range:%f\n",k,uwb_data(uwb_index,k+1));
				}

				logger_ptr->addPlotEvent("uwb", "uwb-" + std::to_string(k), uwb_data(uwb_index, k + 1));
			}
			Eigen::VectorXd diag_noise;
			diag_noise.resize(valid_range.size());
			for(int s=0;s<diag_noise.rows();++s){
				diag_noise(s) = 0.6;
			}

			robust_graph.add(
					UwbFactor(X(pose_counter),
					          valid_range,
					          noiseModel::Diagonal::Sigmas(diag_noise),
//                              range_noise,
                              valid_beacon_set));
			uwb_index++;

		}
		try {
			isam2.update(graph, values);
			isam2.update();
		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;
		}

		try {
			robust_isam2.update(robust_graph, robust_values);
			robust_isam2.update();
		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;
		}


		graph.resize(0);
		values.clear();

		robust_graph.resize(0);
		robust_values.clear();

		initial_pose = isam2.calculateEstimate().at<Pose2>(X(pose_counter));
		logger_ptr->addTraceEvent("trace", "real_time",
		                          Eigen::Vector2d(initial_pose.x(), initial_pose.y()));

		robust_initial_pose = robust_isam2.calculateEstimate().at<Pose2>(X(pose_counter));
//		logger_ptr->addTraceEvent("trace", "robust_real_time", Eigen::Vector2d(
//				robust_initial_pose.x(), robust_initial_pose.y()
//		));


	}

	for (int i = 0; i < pose_counter; ++i) {
		initial_pose = isam2.calculateBestEstimate().at<Pose2>(X(i));
		logger_ptr->addTraceEvent("trace", "final_result",
		                          Eigen::Vector2d(initial_pose.x(), initial_pose.y()));
		initial_pose = robust_isam2.calculateBestEstimate().at<Pose2>(X(i));
//		logger_ptr->addTraceEvent("trace", "robust_final_result",
//		                          Eigen::Vector2d(initial_pose.x(), initial_pose.y()));


	}


	logger_ptr->outputAllEvent(true);


}



