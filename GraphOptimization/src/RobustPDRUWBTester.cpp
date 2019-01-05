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

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"


#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"


#include "OwnEdge/DistanceEdgeSE2Point2.h"

using namespace std;



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

	int pose_counter = 0;
	int beacon_offset = 1000000;

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


	// add prior constraint
	g2o::VertexSE2 *initial_vertex  = new g2o::VertexSE2();
	initial_vertex->setId(pose_counter);
	initial_vertex->setEstimate(Eigen::Vector3d(0.15,5.0,0.0));
	initial_vertex->setFixed(true);

	globalOptimizer.addVertex(initial_vertex);


	//add beacon vertex
	for (int i = 0; i < beacon_set.rows(); ++i) {
		g2o::VertexPointXY* pointXY = new g2o::VertexPointXY();
		pointXY->setId(beacon_offset+i);
		pointXY->setEstimate(Eigen::Vector2d(beacon_set(i,0),beacon_set(i,1)));
		pointXY->setFixed(true);

		globalOptimizer.addVertex(pointXY);
	}

	double latest_pose_2d[3];
	initial_vertex->getEstimateData(latest_pose_2d);



	int uwb_index = 0;
	for(int i=0;i<pdr_data.rows();++i){
		pose_counter++;
		g2o::VertexSE2 *pose_vertex = new g2o::VertexSE2();

		pose_vertex->setId(pose_counter);
		pose_vertex->setEstimate(Eigen::Vector3d(latest_pose_2d[0],latest_pose_2d[1],latest_pose_2d[2]));

		globalOptimizer.addVertex(pose_vertex);


		g2o::EdgeSE2 *edgeSE2 = new g2o::EdgeSE2();
		edgeSE2->vertices()[0] = globalOptimizer.vertex(pose_counter-1);
		edgeSE2->vertices()[1] = globalOptimizer.vertex(pose_counter);
		edgeSE2->setMeasurement(Eigen::Vector3d(pdr_data(i,1),0.0,pdr_data(i,3)));

		Eigen::Matrix3d info = Eigen::Matrix3d::Identity()/0.5;
		info(3,3) = 1.0/(0.1);
		edgeSE2->setInformation(info);

		globalOptimizer.addEdge(edgeSE2);

		while(uwb_data(uwb_index,0) < pdr_data(i,0)){
			DistanceEdge2D* edge_dis_2d = new DistanceEdge2D();
			edge_dis_2d->vertices()[0] = globalOptimizer.vertex(pose_counter);
			edge_dis_2d->vertices()[1] = globalOptimizer.vertex(pose_counter);
			
			edge_dis_2d->setInformation(Eigen::Matrix<double,1,1>::Identity() * 10.0);
			
			
			std::vector < Eigen::Vector2d> beacon_vec;
			std::vector<double> range_vec;
			for(int k=0;k<beacon_set.rows();++k){
				if(uwb_data(uwb_index,k+1)>0.0){
					beacon_vec.push_back(Eigen::Vector2d(beacon_set(k,0),beacon_set(k ,1)));
					range_vec.push_back(uwb_data(uwb_index,k+1));
				}
			}
			edge_dis_2d->setRealMeasurements(range_vec,beacon_vec);
			
			globalOptimizer.addEdge(edge_dis_2d);
			
			
			
			
		}


	}
	globalOptimizer.initializeOptimization();
	globalOptimizer.optimize(100);
	
	for(int i=0;i<pose_counter;++i){
		g2o::VertexSE2 *v = static_cast<g2o::VertexSE2*>(globalOptimizer.vertex(i));
		double data[3];
		v->getEstimateData(data);
		logger_ptr->addTraceEvent("trace","final",Eigen::Vector2d(data[0],data[1]));
	}
	
	
	
	logger_ptr->outputAllEvent(true);


}
