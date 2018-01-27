//
// Created by steve on 18-1-24.
//

#include <iostream>
#include <fstream>


#include <thread>

#include "AWF.h"

#include <Eigen/Dense>


#include "../BayesFilter/KalmanFilter/IMUWBKF.h"

namespace plt = matplotlibcpp;

int main() {

    auto imu_reader = AWF::FileReader("./test/imu.csv");
    auto ground_truth_reader = AWF::FileReader("./test/groundtruth.csv");


    Eigen::MatrixXd imu_data = imu_reader.extractDoulbeMatrix(",");
    Eigen::MatrixXd tmp_imu_data(imu_data.rows(), imu_data.cols());
    tmp_imu_data.block(0, 0, imu_data.rows(), 1) =
            imu_data.block(0, 0, imu_data.rows(), 1) * 1e-9;
    tmp_imu_data.block(0, 1, imu_data.rows(), 3) =
            imu_data.block(0, 4, imu_data.rows(), 3);
    tmp_imu_data.block(0, 4, imu_data.rows(), 3) =
            imu_data.block(0, 1, imu_data.rows(), 3);
    /**
     * Here imu data is:
     * time[s] acc_x[m/s^2] acc_y[m/s^2] acc_z[m/s^2] gyr_x[rad/s] gyr_y[rad/s] gyr_z[rad/s]
     */
    imu_data = tmp_imu_data;


    /**
     * Here ground truth data is:
     * #timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
     * q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
     * v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
     * b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
     * b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]
     */
    Eigen::MatrixXd ground_truth_data = ground_truth_reader.extractDoulbeMatrix(",");
    ground_truth_data.block(0, 0, ground_truth_data.rows(), 1) *= 1e-9;

    if (true) {
        std::vector<double> time;
        std::vector<std::vector<double>> acc_vec_vec = {{},
                                                        {},
                                                        {}};
        std::vector<std::vector<double>> gyr_vec_vec = {{},
                                                        {},
                                                        {}};

        for (int i(0); i < imu_data.rows(); ++i) {
            time.push_back(imu_data(i, 0));
            for (int j(0); j < 3; ++j) {
                acc_vec_vec[j].push_back(imu_data(i, j + 1));
                gyr_vec_vec[j].push_back(imu_data(i, j + 4));
            }
//        std::cout << imu_data.block(i,0,1,7);
        }

        plt::figure();
        for (int i(0); i < 3; ++i) {

            plt::named_plot("acc:" + std::to_string(i), time, acc_vec_vec[i]);
        }

        plt::title("acc");
        plt::legend();
        plt::grid(true);
        plt::save("test.jpg");


        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot("gyr:" + std::to_string(i),
                            time, gyr_vec_vec[i]);
        }
        plt::title("gyr");
        plt::legend();
        plt::grid(true);
        plt::save("te.jpg");


        // plot ground truth path
        std::vector<double> g_time;
        std::vector<std::vector<double>> pos_vec_vec = {{},
                                                        {},
                                                        {}};
        for (int i(0); i < ground_truth_data.rows(); ++i) {
            g_time.push_back(ground_truth_data(i, 0));
            for (int j(0); j < 3; ++j) {
                pos_vec_vec[j].push_back(ground_truth_data(i, j + 1));
            }
        }

        plt::figure();
        for (int i(0); i < 3; ++i) {
            plt::named_plot("pos:" + std::to_string(i),
                            g_time, pos_vec_vec[i]);
        }
        plt::title("ground truth");
        plt::legend();
        plt::grid(true);

        plt::figure();
        plt::title("2d trace");
        plt::plot(pos_vec_vec[0], pos_vec_vec[1]);
        plt::grid(true);

    }

    auto process_noise_vec = Eigen::Matrix<double, 6, 1>::Ones();
    process_noise_vec.block(0, 0, 3, 1) = Eigen::Vector3d(1.0, 1.0, 1.0) * 2.0e-3;
    process_noise_vec.block(3, 0, 3, 1) = Eigen::Vector3d(1.0, 1.0, 1.0) * 1.6968e-4;

    auto measurment_noise_vec = Eigen::Matrix<double, 6, 1>::Ones();
    measurment_noise_vec = measurment_noise_vec * 0.1;

    auto initial_probability = Eigen::Matrix<double, 9, 1>::Ones();
    initial_probability = initial_probability * 0.1;


    auto iuFilter = BSE::IMUWBKFBase<6, double>(process_noise_vec,
                                                measurment_noise_vec,
                                                initial_probability);


    plt::show(true);
    return 1;


}