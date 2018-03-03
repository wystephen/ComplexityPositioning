//
// Created by steve on 18-1-19.
//


#include <iostream>
#include <fstream>


#include <thread>
#include <AWF.h>


#include "AWF.h"


#include "BSE.h"


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

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "../include/OwnEdge/DistanceEdge.h"
#include "../include/OwnEdge/DistanceEdge.cpp"


namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {

    std::cout.precision(30);
    // parameters
    std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";


    // 3 300 0.2 5.0 10000 0.2 5.0 5
    int only_method = 3;
    int only_particle_num = 1150;
    double only_transpose_sigma = 0.3;
    double only_eval_sigma = 5.0;

    int fus_particle_num = 30000;
    double fus_transpose_sigma = 1.3;
    double fus_eval_sigma = 1.0;



    /// g2o parameter

    double first_info = 1000.0;
    double second_info = 1000.0;


    double distance_info = 1.0;
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


    auto imu_tool = BSE::ImuTools();


    //process
    imu_tool.processImuData(left_imu_data);
    imu_tool.processImuData(right_imu_data);
    imu_tool.processImuData(head_imu_data);

    Eigen::MatrixXd process_noise_matrix =
            Eigen::MatrixXd::Identity(6, 6);
    process_noise_matrix.block(0, 0, 3, 3) *= 0.1;
    process_noise_matrix.block(3, 3, 3, 3) *= (0.1 * M_PI / 180.0);

    Eigen::MatrixXd measurement_noise_matrix =
            Eigen::MatrixXd::Identity(uwb_data.cols() - 1, uwb_data.cols() - 1);
    measurement_noise_matrix *= 0.1;


    Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
    initial_prob_matrix.block(0, 0, 3, 3) *= 0.001;
    initial_prob_matrix.block(3, 3, 3, 3) *= 0.001;
    initial_prob_matrix.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);

    int left_index(5), right_index(5), head_index(5), uwb_index(0);
    int last_left_index(0), last_right_index(0), last_head_index(0), last_uwb_index(0);
    BSE::IMUWBKFBase left_imu_ekf(initial_prob_matrix);
    BSE::IMUWBKFBase right_imu_ekf(initial_prob_matrix);
    Eigen::Isometry3d left_last_T = (Eigen::Isometry3d::Identity());// last transform matrix.
    Eigen::Isometry3d right_last_T = (Eigen::Isometry3d::Identity());
    int left_last_zv_flag(false), right_last_zv_flag(false);

//    left_imu_ekf.setTime_interval_((left_imu_data(left_imu_data.rows() - 1, 0) - left_imu_data(0, 0)) /
//                                   double(left_imu_data.rows()));
//    right_imu_ekf.setTime_interval_((right_imu_data(right_imu_data.rows() - 1, 0) - right_imu_data(0, 0))
//                                    / double(right_imu_data.rows()));
    // IMU initial lambda func
    auto local_imu_initial_func =
            [&process_noise_matrix,
                    & measurement_noise_matrix,
                    & initial_prob_matrix]
                    (
                            BSE::IMUWBKFBase &imu_ekf,
                            Eigen::MatrixXd initial_input
                    ) {
                /**
                 *  initial_input 10 * 6 ...
                 */
                imu_ekf.initial_state(initial_input,
                                      0.0,
                                      Eigen::Vector3d(0, 0, 0));

            };
    // IMU update lambda function
    auto local_imu_update_func =
            [&process_noise_matrix,
                    & measurement_noise_matrix,
                    & initial_prob_matrix]
                    (BSE::IMUWBKFBase &imu_ekf,
                     Eigen::MatrixXd input) {
                imu_ekf.StateTransaction(
                        input, process_noise_matrix,
                        BSE::StateTransactionMethodType::NormalRotation
                );
            };

    // IMU zero-velocity correct
    auto local_imu_zupt_func =
            [&measurement_noise_matrix]
                    (
                            BSE::IMUWBKFBase &imu_ekf
                    ) {
                imu_ekf.MeasurementState(
                        Eigen::Vector3d(0, 0, 0),
                        Eigen::Matrix3d::Identity() * 0.00025,
                        BSE::MeasurementMethodType::NormalZeroVeclotiMeasurement
                        ///
                );
            };


    local_imu_initial_func(left_imu_ekf, left_imu_data.block(0, 1, 20, 6));
    local_imu_initial_func(right_imu_ekf, right_imu_data.block(0, 1, 20, 6));

    left_last_T = left_imu_ekf.getTransformMatrix();
    right_last_T = right_imu_ekf.getTransformMatrix();



    /**
     * Initial graph
     */
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolverX SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initial solver
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
    linearSolver->setWriteDebug(true);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);

    int beacon_index_offset(0);
    for(int k(0);k<beacon_set_data.rows();k++){
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        double *d = new double[6];
        for(int ki(0);ki<3;++ki){
            d[ki] = beacon_set_data(k,ki);
        }
        v->setEstimateData(d);
//        v->setFixed(true);

    }




    // set left vertex index and right vertex index
    int left_vertex_index_init = 1000000;
    int right_vertex_index_init = 2000000;
    int uwb_vertex_index_init = 3000000;
    int left_vertex_index = left_vertex_index_init;
    int right_vertex_index = right_vertex_index_init;
    int uwb_vertex_index = uwb_vertex_index_init;




    /**
     * Aux tool
     */
    std::vector<std::vector<double>> left_trace = {{},
                                                   {},
                                                   {}};
    std::vector<std::vector<double>> right_trace = {{},
                                                    {},
                                                    {}};
    std::vector<std::vector<double>> uwb_trace = {{},
                                                  {},
                                                  {}};
    std::vector<std::vector<double>> fusiong_trace = {{},
                                                      {},
                                                      {}};


    auto add_foot_vertex = [&globalOptimizer,
    &first_info,
    &second_info]
            (Eigen::Isometry3d transform_matrix,
             int current_index,
             bool add_edge) {
        g2o::VertexSE3 *foot_se3 = new g2o::VertexSE3();
        foot_se3->setId(current_index);
        globalOptimizer.addVertex(foot_se3);

        if (add_edge) {
            auto *e = new g2o::EdgeSE3();

            e->vertices()[0] = globalOptimizer.vertex(current_index - 1);
            e->vertices()[1] = globalOptimizer.vertex(current_index);
            Eigen::Matrix<double, 6, 6> information(Eigen::Matrix<double, 6, 6>::Identity());

            information(0, 0) = information(1, 1) = information(2, 2) = first_info;
            information(3, 3) = information(4, 4) = information(5, 5) = second_info;

//            if (is_corner) {
//                information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
//                information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
//            }
            e->setInformation(information);
            e->setMeasurement(transform_matrix);
            globalOptimizer.addEdge(e);
        }


    };


    auto add_uwb_edge = [&globalOptimizer,
    &left_vertex_index,
    &right_vertex_index,
    &distance_info]
            (double measurement,
             int measurement_index,
            int uwb_index){
        Eigen::Matrix<double,1,1> info_matrix;
        info_matrix(0,0) = distance_info;

        auto *edge = new DistanceEdge();
        edge->vertices()[0] = globalOptimizer.vertex(measurement_index);
        edge->vertices()[1] = globalOptimizer.vertex(left_vertex_index-1);
        edge->setMeasurement(measurement);
        edge->setInformation(info_matrix);

        globalOptimizer.addEdge(edge);

        auto *edge_right = new DistanceEdge();
        edge_right->vertices()[0] = globalOptimizer.vertex(measurement_index);
        edge_right->vertices()[1] = globalOptimizer.vertex(right_vertex_index-1);
        edge_right->setMeasurement(measurement);
        edge_right->setInformation(info_matrix);

        globalOptimizer.addEdge(edge_right);

    };


    /**
     * Main loop add foot ,
     */
    while (true) {
        // end condition.
        if (left_index + 5 >= left_imu_data.rows() ||
            right_index + 5 >= right_imu_data.rows() ||
            head_index + 5 >= head_imu_data.rows() ||
            uwb_index + 2 >= uwb_data.rows()) {
            break;
        }


        if (left_imu_data(left_index, 0) < uwb_data(uwb_index, 0)) {
            ///update left index
//            std::cout << "left foot" ;//<< std::endl;
            bool zv_flag =
                    imu_tool.GLRT_Detector(
                            left_imu_data.block(left_index - 5, 1, 10, 6)) > 0.5 ? true : false;

            // non-zero velocity to zero velocity
            if (zv_flag && !left_last_zv_flag) {


            }

            // zero velcity to non-zero velocity
            if (!zv_flag && left_last_zv_flag) {
                auto the_transform = left_imu_ekf.getTransformMatrix();
                // Add vertex and edge
                for (int k(0); k < 3; ++k) {
                    left_trace[k].push_back(the_transform(k, 3));
                }
                add_foot_vertex(left_last_T.inverse()*the_transform,
                left_vertex_index,
                left_vertex_index>left_vertex_index_init);
                left_vertex_index++;


                left_last_T = the_transform;
            }


            // ZUPT state or non-ZUPT state
            if (zv_flag) {
                local_imu_zupt_func(left_imu_ekf);

            } else {
                local_imu_update_func(left_imu_ekf,
                                      left_imu_data.block(left_index, 1, 1, 6).transpose());
            }
            left_last_zv_flag = zv_flag;
            left_index++;
        }

        if (right_imu_data(right_index, 0) < uwb_data(uwb_index, 0)) {
            ///update right index
//            std::cout << "right foot";//<< std::endl;

            bool zv_flag =
                    imu_tool.GLRT_Detector(
                            right_imu_data.block(right_index - 5, 1, 10, 6)
                    ) > 0.5 ? true : false;

            //non-zero velocity to zero velocity
            if (!right_last_zv_flag && zv_flag) {

            }

            // zero velocity to non-zero velocity
            if (right_last_zv_flag && !zv_flag) {
                auto the_transform = right_imu_ekf.getTransformMatrix();

                for (int k(0); k < 3; ++k) {
                    right_trace[k].push_back(the_transform(k, 3));
                }
                add_foot_vertex(right_last_T.inverse()*the_transform,
                right_vertex_index,
                right_vertex_index>right_vertex_index_init);
                right_vertex_index++;

                right_last_T = the_transform;
            }

            //ZUPT state or non-ZUPT state
            if (zv_flag) {
                local_imu_zupt_func(right_imu_ekf);
            } else {
                local_imu_update_func(right_imu_ekf,
                                      right_imu_data.block(right_index, 1, 1, 6).transpose());
            }

            right_last_zv_flag = zv_flag;
            right_index++;
        }

        if (uwb_data(uwb_index, 0) <= right_imu_data(right_index, 0) &&
            uwb_data(uwb_index, 0) <= left_imu_data(left_index, 0)) {
            /// update uwb index
//            std::cout << "uwb" << std::endl;
//            add_uwb_edge(
//                    uw
//
//            );
            for(int k(1);k<uwb_data.cols();++k){
                if(uwb_data(uwb_index,k)>0){
                    std::cout << uwb_index
                              << ","
                              << uwb_data(uwb_index,k)
                              << std::endl;

                    add_uwb_edge(uwb_data(uwb_index,k),k-1,0);
                }

            }

            uwb_index++;

        }


    }

    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);
    globalOptimizer.optimize(1000);


    // get pose
    std::vector<std::vector<double>> graph_left={{},{},{}};
    std::vector<std::vector<double>> graph_right={{},{},{}};
    for(int i(left_vertex_index_init);i<left_vertex_index;++i){

        double * data = new double[10];
       globalOptimizer.vertex(i)->getEstimateData(data);
        for(int j(0);j<3;++j){
            graph_left[j].push_back(data[j]);
        }
    }

    for(int i(right_vertex_index_init);i<right_vertex_index;++i){
        double * data = new double[10];
        globalOptimizer.vertex(i)->getEstimateData(data);
        for(int j(0);j<3;++j){
            graph_right[j].push_back(data[j]);
        }
    }

    plt::figure();
    plt::title("result");
//    std::cout << " left" << std::endl;
    plt::named_plot("left", left_trace[0], left_trace[1], "-+");
//    std::cout << "right" << std::endl;
    plt::named_plot("right", right_trace[0], right_trace[1], "-+");
//    std::cout << "grid " << std::endl;
    plt::named_plot("left_graph",graph_left[0],graph_left[1],"-*");
    plt::named_plot("right_graph",graph_right[0],graph_right[1],"-*");

    plt::grid(true);
    plt::legend();
    plt::show();
}
