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

#include "../BayesFilter/KalmanFilter/IMUWBKF.h"
#include "../BayesFilter/KalmanFilter/IMUWBKF.cpp"

#include "../AuxiliaryTool/UwbTools.h"
#include "../AuxiliaryTool/UwbTools.cpp"

#include "../AuxiliaryTool/ImuTools.h"
#include "../AuxiliaryTool/ImuTools.cpp"


namespace plt = matplotlibcpp;


int main(int argc, char *argv[]) {


    std::cout.precision(30);
    // parameters
    std::string dir_name = "/home/steve/Data/FusingLocationData/0010/";



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

    Eigen::MatrixXd measurement_noise_matrix =
            Eigen::MatrixXd::Identity(uwb_data.cols() - 1, uwb_data.cols() - 1);
    measurement_noise_matrix *= 0.1;


    Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
    initial_prob_matrix.block(0, 0, 3, 3) *= 0.001;
    initial_prob_matrix.block(3, 3, 3, 3) *= 0.001;
    initial_prob_matrix.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);


    auto f = [&process_noise_matrix,
            &initial_prob_matrix,
            &measurement_noise_matrix,
            &uwb_data,
            &beacon_set_data,
            &initial_pos,
            &initial_ori,
            &optimize_trace_vec,
            &imu_tool](const Eigen::MatrixXd &imu_data,
                       std::string data_name) {
        auto filter = BSE::IMUWBKFBase(
                initial_prob_matrix);
        filter.setTime_interval_(0.005);
//    filter.IS_DEBUG = true;


        auto time_begin = AWF::getDoubleSecondTime();
        filter.initial_state(imu_data.block(10, 1, 100, 6),
                             initial_ori,
                             initial_pos);
        std::cout << "costed time :" << AWF::getDoubleSecondTime() - time_begin
                  << std::endl;

        std::vector<std::vector<double>> pose = {{},
                                                 {},
                                                 {}};
        std::vector<std::vector<double>> velocity = {{},
                                                     {},
                                                     {}};
        std::vector<std::vector<double>> angle = {{},
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
            double uwb_index = 0;
            /// uwb measurement
            bool tmp_break_flag = false;
            while (uwb_data(uwb_index, 0) < imu_data(i, 0)) {
                uwb_index++;
                if (uwb_index == uwb_data.rows()) {
//                    break;

//                    return;
//                    tmp_break_flag = true;
                    uwb_index--;
                    break;

                }
            }
//            if(tmp_break_flag)
//            {
//                i = imu_data.rows()+10;
//                break;
//            }
            if (uwb_data(uwb_index, 0) - imu_data(i, 0) < 0.01) {
//                filter.MeasurementState(uwb_data.block(uwb_index, 1, 1, uwb_data.cols() - 1),
//                                        measurement_noise_matrix,
//                                        BSE::MeasurementMethodType::NormalUwbMeasuremnt);
//                std::cout << "test uwb :" << uwb_data(uwb_index, 0) << ",imu :"
//                          << imu_data(i, 0) << std::endl;

                for (int k(1); k < uwb_data.cols(); ++k) {
                    if (uwb_data(uwb_index, k) > 0 && uwb_data(uwb_index, k) < 15.0) {
                        Eigen::Vector4d measurement_data(0, 0, 0, uwb_data(uwb_index, k));
                        measurement_data.block(0, 0, 3, 1) = beacon_set_data.block(k - 1, 0, 1, 3).transpose();
                        measurement_noise_matrix.resize(1, 1);
                        measurement_noise_matrix(0, 0) = 0.003;
                        // correct
//                        filter.MeasurementState(measurement_data,
//                                                measurement_noise_matrix,
//                                                BSE::MeasurementMethodType::NormalUwbMeasuremnt);

                    }
                }
            }


            if (imu_tool.GLRT_Detector(imu_data.block(i - 4, 1, 7, 6))) {
                /// zero velocity detector
                filter.MeasurementState(Eigen::Vector3d(0, 0, 0),
                                        Eigen::Matrix3d::Identity() * 0.000251001,
                                        BSE::MeasurementMethodType::NormalZeroVeclotiMeasurement);
//                filter.MeasurementState(imu_data.block(i, 4, 1, 3).transpose(),
//                                        Eigen::Matrix3d::Identity() * 0.0001,
//                                        BSE::MeasurementMethodType::NormalAngleConstraint);
                if (zv_flag.size()>3 &&
                zv_flag.at(zv_flag.size() - 2) < 0.5) {
                    std::cout << " linear accc:"
                              << (filter.getRotate_q().toRotationMatrix() *
                                  imu_data.block(i, 1, 1, 3).transpose()).transpose()
                              << std::endl;
                }

                zv_flag.push_back(1.0);
            } else {
                zv_flag.push_back(0.0);
            }

            Eigen::VectorXd state = filter.getState_();
//        std::cout << state.transpose() << std::endl;
            for (int j(0); j < 3; ++j) {
                pose[j].push_back(state(j));
                velocity[j].push_back(state(j + 3));
                angle[j].push_back(state(j + 6));
                acc[j].push_back(imu_data(i, j + 1));
                angle_velocity[j].push_back(imu_data(i, j + 4));
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
        plt::named_plot("ekf trace", pose[0], pose[1], "-.");
        plt::named_plot("optimized trace",
                        optimize_trace_vec[0],
                        optimize_trace_vec[1], "*");

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
        plt::xlim(min_v, max_v);
        plt::ylim(min_v, max_v);

        plt::legend();
        plt::grid(true);
        plt::title(data_name + "trace");

    };

    f(left_imu_data, "left_foot");
//    f(right_imu_data, "right_foot");
//    f(head_imu_data, "head");

    plt::show();

}