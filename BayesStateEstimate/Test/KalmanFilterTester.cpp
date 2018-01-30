//
// Created by steve on 18-1-24.
//


#include <iostream>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "AWF.h"

#include "../BayesFilter/KalmanFilter/IMUWBKF.h"
#include "../BayesFilter/KalmanFilter/IMUWBKF.cpp"


/**
 *  process the imu data according to the typical sensor model
 * @param imu_data after preprocess time[s] acc_(x,y,z)[m*s^-2] gyr_(x,y,z)[rad*s^-1] mag_(x,y,z) pressure
 */
void processImuData(Eigen::MatrixXd &imu_data) {
//    Eigen::MatrixXd tmp_data = imu_data.;
    Eigen::MatrixXd tmp_data(imu_data);

    int row(imu_data.rows());
    int col(imu_data.cols());
    imu_data.resize(row, 1 + 3 + 3 + 3 + 1);//
    // time
    imu_data.block(0, 0, row, 1) = tmp_data.block(0, 0, row, 1) * 1.0;
    imu_data.block(0, 1, row, 3) = tmp_data.block(0, 2, row, 3) * 9.81;
    imu_data.block(0, 4, row, 3) = tmp_data.block(0, 5, row, 3) * (M_PI / 180.0);
    imu_data.block(0, 7, row, 3) = tmp_data.block(0, 8, row, 3) * 1.0;
    return;
}

int main(int argc, char *argv[]) {
    // parameters
    std::string dir_name = "/home/steve/Data/FusingLocationData/0014/";



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

    //process
    processImuData(left_imu_data);
    processImuData(right_imu_data);
    processImuData(head_imu_data);

    Eigen::MatrixXd process_noise_matrix =
            Eigen::MatrixXd::Identity(6, 6);
    process_noise_matrix.block(0, 0, 3, 3) *= 0.01;
    process_noise_matrix.block(3, 3, 3, 3) *= (0.01 * M_PI / 180.0);

    Eigen::MatrixXd measurement_noise_matrix = Eigen::MatrixXd::Identity(uwb_data.cols() - 1, uwb_data.cols() - 1);
    measurement_noise_matrix *= 0.1;


    Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
    initial_prob_matrix.block(0, 0, 3, 3) *= 0.1;
    initial_prob_matrix.block(3, 3, 3, 3) *= 0.01;
    initial_prob_matrix.block(6, 6, 3, 3) *= 0.1 * (M_PI / 180.0);


    auto time_begin = AWF::getDoubleSecondTime();
    auto filter = BSE::IMUWBKFBase(
            initial_prob_matrix);


    filter.initial_state(head_imu_data.block(0, 1, 100, 6));
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

//    filter.sett
    for (int i(0); (left_imu_data(i, 0) - left_imu_data(0, 0)) < 2.0; ++i) {
        filter.StateTransaction(left_imu_data.block(i,1,1,6).transpose(),
                                process_noise_matrix,
        BSE::IMUMethodType::NormalRotation);

    }


}