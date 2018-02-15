//
// Created by steve on 18-1-19.
//


#include <iostream>
#include <fstream>


#include <thread>


#include "AWF.h"


#include "../../BayesStateEstimate/AuxiliaryTool/ImuTools.h"
#include "../../BayesStateEstimate/AuxiliaryTool/UwbTools.h"

#include "../../BayesStateEstimate/BayesFilter/KalmanFilter/IMUWBKF.h"
#include "../../BayesStateEstimate/BayesFilter/KalmanFilter/IMUWBKF.cpp"


int main(int argc, char *argv[]) {
    std::cout.precision(30);
    // parameters
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

    int left_index(5), right_index(5), head_index(5), uwb_index(0);
    int last_left_index(0), last_right_index(0), last_head_index(0), last_uwb_index(0);
    BSE::IMUWBKFBase left_imu_ekf(initial_prob_matrix);
    BSE::IMUWBKFBase right_imu_ekf(initial_prob_matrix);
    Eigen::Isometry3d left_last_T(Eigen::Isometry3d::Identity());// last transform matrix.
    Eigen::Isometry3d right_last_T(Eigen::Isometry3d::Identity());
    int left_last_zv_flag(true), right_last_zv_flag(true);

    left_imu_ekf.setTime_interval_((left_imu_data(left_imu_data.row() - 1, 0) - left_imu_data(0, 0)) /
                                   double(left_imu_data.rows()));
    right_imu_ekf.setTime_interval_((right_imu_data(right_imu_data.rows() - 1, 0) - right_imu_data(0, 0))
                                    / double(right_imu_data.rows()));

    /**
     * Main loop add foot ,
     */
    while (1) {
        // end condition.
        if (left_index + 5 >= left_imu_data.rows() ||
            right_index + 5 >= right_imu_data.rows() ||
            head_index + 5 >= head_imu_data.rows() ||
            uwb_index + 2 >= uwb_data.rows()) {
            break;
        }
        // IMU update lambda func
        auto local_imu_update_func =
                [&process_noise_matrix,
                        & measurement_noise_matrix,
                        & initial_prob_matrix]
                        (BSE::IMUWBKFBase &imu_ekf,
                         Eigen::MatrixXd &input) {
                    imu_ekf.StateTransaction(
                            input, process_noise_matrix,
                            BSE::StateTransactionMethodType::NormalRotation
                    );
                };
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
                    );
                };


        if (left_imu_data(left_index, 0) < uwb_data(uwb_index, 0)) {
            //update left index
            bool zv_flag =
                    imu_tool.GLRT_Detector(
                            left_imu_data.block(left_index - 5, 1, 10, 6)) > 0.5 ? true : false;
            // non-zero velocity to zero velocity
            if (zv_flag && !left_last_zv_flag) {

            }

            if (!zv_flag && left_last_zv_flag) {

            }


            local_imu_update_func(left_imu_ekf,
                                  left_imu_data.block(left_index, 1, 1, 6).transpose());

            left_last_zv_flag = zv_flag;
            left_index++;


        }
        if (right_imu_data(right_index, 0) < uwb_data(uwb_index, 0)) {
            //update right index

        }
        if (uwb_data(uwb_index, 0) < right_imu_data(right_index, 0) &&
            uwb_data(uwb_index, 0) < left_imu_data(left_index, 0)) {
            // update uwb index

        }


    }


}
