//
// Created by steve on 18-1-24.
//


#include <iostream>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <AWF.h>dd


#include "AWF.h"

#include "BayesFilter/KalmanFilter/IMUWBKFSimple.h"
#include "BayesFilter/KalmanFilter/IMUWBKFSimple.cpp"


#include "BayesFilter/KalmanFilter/KFComplex.h"

#include "../AuxiliaryTool/UwbTools.h"
#include "../AuxiliaryTool/UwbTools.cpp"

#include "../AuxiliaryTool/ImuTools.h"
#include "../AuxiliaryTool/ImuTools.cpp"


namespace plt = matplotlibcpp;


int main(int argc, char *argv[]) {


    std::cout.precision(10);
    // parameters
    std::string dir_name = "/home/steve/Data/NewFusingLocationData/0015/";



    // load data
    AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
            right_foot_file(dir_name + "RIGHT_FOOT.data"),
            head_imu_file(dir_name + "HEAD.data");
//            uwb_file(dir_name + "uwb_result.csv"),
//            beacon_set_file(dir_name + "beaconSet.csv");

    Eigen::MatrixXd left_imu_data = left_foot_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd right_imu_data = right_foot_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd head_imu_data = head_imu_file.extractDoulbeMatrix(",");
//    Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
//    Eigen::MatrixXd beacon_set_data = beacon_set_file.extractDoulbeMatrix(",");

//    assert(beacon_set_data.rows() == (uwb_data.cols() - 1));

    // get the initial pose based on uwb data.

//    std::cout << uwb_data.block(0, 0, 1, uwb_data.cols()) << std::endl;

//    auto uwb_tool = BSE::UwbTools(uwb_data,
//                                  beacon_set_data);

//    Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();
    Eigen::Vector3d initial_pos = Eigen::Vector3d(0, 0, 0);
    double initial_ori = 0.0;

    std::vector<std::vector<double>> optimize_trace_vec = {{},
                                                           {},
                                                           {}};



//    std::cout << last_uwb_err
    auto imu_tool = BSE::ImuTools();


    //process
    imu_tool.processImuData(left_imu_data);
    imu_tool.processImuData(right_imu_data);
    imu_tool.processImuData(head_imu_data);

    Eigen::MatrixXd process_noise_matrix =
            Eigen::MatrixXd::Identity(6, 6);
    process_noise_matrix.block(0, 0, 3, 3) *= 0.1;
    process_noise_matrix.block(3, 3, 3, 3) *= (0.1 * M_PI / 180.0);


    Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
    initial_prob_matrix.block(0, 0, 3, 3) *= 0.001;
    initial_prob_matrix.block(3, 3, 3, 3) *= 0.001;
    initial_prob_matrix.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);
//    initial_prob_matrix.block(9, 9, 3, 3) *= 0.001;
//    initial_prob_matrix.block(12, 12, 3, 3) *= 0.001 * (M_PI / 180.0);
    Eigen::MatrixXd initial_prob_matrix_complex = Eigen::MatrixXd::Identity(15, 15);
    initial_prob_matrix_complex.block(0, 0, 3, 3) *= 0.001;
    initial_prob_matrix_complex.block(3, 3, 3, 3) *= 0.001;
    initial_prob_matrix_complex.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);
    initial_prob_matrix_complex.block(9, 9, 3, 3) *= 0.001;
    initial_prob_matrix_complex.block(12, 12, 3, 3) *= 0.001 * (M_PI / 180.0);

    auto f = [&process_noise_matrix,
            &initial_prob_matrix,
            &initial_prob_matrix_complex,
            &initial_pos,
            &initial_ori,
            &optimize_trace_vec,
            &imu_tool](const Eigen::MatrixXd &imu_data,
                       std::string data_name) {
        auto filter = BSE::IMUWBKFSimple(
                initial_prob_matrix);

        auto filter_complex = BSE::KFComplex(initial_prob_matrix);

        double tmp_time_interval = (imu_data(imu_data.rows() - 1, 0) - imu_data(0, 0))
                                   / double(imu_data.rows());
        std::cout << "time interval :" << tmp_time_interval << std::endl;
        if (std::abs(tmp_time_interval - 0.005) < 0.0001) {
            filter.setTime_interval_(0.005);
            filter_complex.time_interval_ = 0.005;
        } else if (std::abs(tmp_time_interval - 0.01) < 0.001) {
            filter.setTime_interval_(0.01);
            filter_complex.time_interval_ = 0.01;
        } else {
            filter.setTime_interval_(tmp_time_interval);
            filter_complex.time_interval_ = tmp_time_interval;
        }
        filter.setLocal_g_(-9.884);
        filter_complex.local_g_ = -9.884;
//    filter.IS_DEBUG = true;


        auto time_begin = AWF::getDoubleSecondTime();
        filter.initial_state(imu_data.block(10, 1, 100, 6),
                             initial_ori,
                             initial_pos);


        filter_complex.initial_state(imu_data.block(10, 1, 100, 9),
                                     initial_ori,
                                     initial_pos);
        std::cout << "costed time :" << AWF::getDoubleSecondTime() - time_begin
                  << std::endl;
        std::vector<std::vector<double>> pose_simple = {{},
                                                        {},
                                                        {}};
        std::vector<std::vector<double>> pose = {{},
                                                 {},
                                                 {}};
        std::vector<std::vector<double>> velocity = {{},
                                                     {},
                                                     {}};
        std::vector<std::vector<double>> angle = {{},
                                                  {},
                                                  {}};

        std::vector<std::vector<double>> mag = {{},
                                                {},
                                                {}};
        std::vector<double> zv_flag = {};

        std::vector<std::vector<double>> angle_velocity = {{},
                                                           {},
                                                           {}};

        std::vector<std::vector<double>> acc = {
                {},
                {},
                {}};

//    filter.sett
        for (int i(5); i < imu_data.rows() - 5; ++i) {
            /// state transaction equation
            filter.StateTransaction(imu_data.block(i, 1, 1, 6).transpose(),
                                    process_noise_matrix,
                                    BSE::StateTransactionMethodType::NormalRotation);


            auto complex_state = filter_complex.StateTransIMU(imu_data.block(i, 1, 1, 6).transpose(),
                                                              process_noise_matrix);

            double uwb_index = 0;
            /// uwb measurement
            bool tmp_break_flag = false;


            if (imu_tool.GLRT_Detector(imu_data.block(i - 5, 1, 10, 6))) {
                /// zero velocity detector
                filter.MeasurementState(Eigen::Vector3d(0, 0, 0),
                                        Eigen::Matrix3d::Identity() * 0.000251001,
                                        BSE::MeasurementMethodType::NormalZeroVeclotiMeasurement);

                filter_complex.MeasurementStateZV(Eigen::Matrix3d::Identity() * 0.00025);

                /// angle constraint through acc.
                double last_diff = std::abs(imu_data.block(i - 1, 1, 1, 3).norm() - 9.884);
                double current_diff = std::abs(imu_data.block(i, 1, 1, 3).norm() - 9.884);
                double next_diff = std::abs(imu_data.block(i + 1, 1, 1, 3).norm() - 9.884);
                int zv_index = zv_flag.size() - 1;
                bool last_zv_flag = true;
                for (int d(0); d < 5; ++d) {
                    if (zv_index - d > 0 && !zv_flag[zv_index - d]) {
                        last_zv_flag = false;
                    }
                }
//                if((imu_data.block(i,1,1,3).transpose()-
//                        Eigen::Vector3d(-1.263,0.5163,9.742)).norm()<0.02)
//                filter.MeasurementState(imu_data.block(i, 1, 1, 3).transpose(),
//                                        Eigen::Matrix3d::Identity() * 0.1 * M_PI / 180.0,
//                                        BSE::MeasurementMethodType::NormalAngleConstraint);


//                filter_complex.MeasurementAngleCorrect(imu_data.block(i, 7, 1, 3).transpose(),
//                                                       Eigen::Matrix3d::Identity() * 0.5);
                Eigen::Matrix<double, 6, 1> tmp_gm;
                tmp_gm.block(0, 0, 3, 1) = imu_data.block(i, 1, 1, 3).transpose();
                tmp_gm.block(3, 0, 3, 1) = imu_data.block(i, 7, 1, 3).transpose();
                Eigen::Matrix<double, 6, 6> cov_matrix = Eigen::Matrix<double, 6, 6>::Identity();
                cov_matrix.block(0, 0, 3, 3) *= 100.5;
                cov_matrix.block(3, 3, 3, 3) *= 5000000000000000.5;


//                if(std::abs(imu_data(i,3)-9.74)<0.01 && current_diff < 0.001)
                filter_complex.MeasurementAngleCorrectMG(tmp_gm, cov_matrix);

                if (zv_flag.size() > 3 &&
                    zv_flag.at(zv_flag.size() - 2) < 0.5) {
                    std::cout << " lacc:"
                              << (filter_complex.rotation_q_.toRotationMatrix() *
                                  imu_data.block(i, 1, 1, 3).transpose()).transpose()
                              << std::endl;
                }

                zv_flag.push_back(1.0);
            } else {
                zv_flag.push_back(0.0);
            }

            Eigen::VectorXd state_simple = filter.getState_();
            Eigen::VectorXd state = filter_complex.state_x_;
//        std::cout << state.transpose() << std::endl;
            for (int j(0); j < 3; ++j) {
                pose_simple[j].push_back(state_simple(j));
                pose[j].push_back(state(j));
                velocity[j].push_back(state(j + 3));
                angle[j].push_back(state(j + 6));
                acc[j].push_back(imu_data(i, j + 1));
                angle_velocity[j].push_back(imu_data(i, j + 4));
                mag[j].push_back(imu_data(i, j + 7));
            }

        }

        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), pose[i]);
        }
        plt::title(data_name + "pose");
        plt::grid(true);
        plt::legend();

        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), mag[i]);
        }
        plt::title(data_name + "mag");
        plt::grid(true);
        plt::legend();


        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), velocity[i]);
        }
        plt::title(data_name + "vel");
        plt::grid(true);
        plt::legend();

        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), angle[i]);
        }
        plt::named_plot("zv_falg", zv_flag);
        plt::title(data_name + "angle");
        plt::grid(true);
        plt::legend();


        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), angle_velocity[i]);
        }
        plt::named_plot("zv_falg", zv_flag);
        plt::title(data_name + "angle velocity");
        plt::grid(true);
        plt::legend();


        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot(std::to_string(i), acc[i]);
        }
        plt::named_plot("zv_falg", zv_flag);
        plt::title(data_name + "acc");
        plt::grid(true);
        plt::legend();

        plt::figure();
        plt::named_plot("ekf trace", pose[0], pose[1], "-+");
        plt::named_plot("simple trace", pose_simple[0], pose_simple[1], "-+");
        plt::named_plot("optimized trace",
                        optimize_trace_vec[0],
                        optimize_trace_vec[1], "*");


        AWF::writeVectorsToCsv("./XsenseResult/ekf.csv", pose);
        AWF::writeVectorsToCsv("./XsenseResult/pose.csv", pose_simple);

        double min_v(0.0), max_v(0.0);
        min_v = std::min(std::min(*std::min_element(pose[0].begin(), pose[0].end()),
                                  *std::min_element(pose[1].begin(), pose[1].end())),
                         std::min(*std::min_element(optimize_trace_vec[0].begin(), optimize_trace_vec[0].end()),
                                  *std::min_element(optimize_trace_vec[1].begin(),
                                                    optimize_trace_vec[1].end())));

        max_v = std::max(std::max(*std::max_element(pose[0].begin(), pose[0].end()),
                                  *std::max_element(pose[1].begin(), pose[1].end())),
                         std::max(*std::max_element(optimize_trace_vec[0].begin(), optimize_trace_vec[0].end()),
                                  *std::max_element(optimize_trace_vec[1].begin(),
                                                    optimize_trace_vec[1].end())));
//        plt::xlim(min_v, max_v);
//        plt::ylim(min_v, max_v);

        plt::legend();
        plt::grid(true);
        plt::title(data_name + "trace");

    };
//
//    f(left_imu_data, "left_foot");
    f(right_imu_data, "right_foot");
//    f(head_imu_data, "head");

    plt::show();

}