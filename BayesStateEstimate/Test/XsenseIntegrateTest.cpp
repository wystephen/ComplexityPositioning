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

    std::string dir_name = "/home/steve/Data/XsensUwb/MTI700/0001/";

    AWF::FileReader imu_file(dir_name + "imu.data");
    AWF::FileReader uwb_file(dir_name + "uwb_data.csv");
    AWF::FileReader beacon_file(dir_name + "beaconset_no_mac.txt");

    Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd beacon_data = beacon_file.extractDoulbeMatrix(",");

    Eigen::MatrixXd process_noise_matrix =
            Eigen::MatrixXd::Identity(6, 6);
    process_noise_matrix.block(0, 0, 3, 3) *= 0.1;
    process_noise_matrix.block(3, 3, 3, 3) *= (0.1 * M_PI / 180.0);


    Eigen::MatrixXd initial_prob_matrix = Eigen::MatrixXd::Identity(9, 9);
    initial_prob_matrix.block(0, 0, 3, 3) *= 0.001;
    initial_prob_matrix.block(3, 3, 3, 3) *= 0.001;
    initial_prob_matrix.block(6, 6, 3, 3) *= 0.001 * (M_PI / 180.0);


    auto filter = BSE::IMUWBKFSimple(initial_prob_matrix);
//    filter.setTime_interval_(0.01);

    filter.setTime_interval_(0.01);
    filter.initial_state(imu_data.block(0, 1, 10, 6), 0.0);
    filter.setLocal_g_(-9.81);
//    filter.IS_DEBUG = true;


    auto imu_tool = BSE::ImuTools();

    std::vector<std::vector<double>> trace = {{},
                                              {},
                                              {}};


    typedef std::vector<std::vector<double>> vec_data;
    vec_data acc = {{},
                    {},
                    {}};
    vec_data gyr = {{},
                    {},
                    {}};
    vec_data velocity = {{},
                         {},
                         {}};
    vec_data attitude = {{},
                         {},
                         {}};


    int uwb_index = 0;


    for (int i(0); i < imu_data.rows(); ++i) {
        filter.StateTransaction(imu_data.block(i, 1, 1, 6).transpose(),
                                process_noise_matrix,
                                BSE::StateTransactionMethodType::NormalRotation);

        auto state_T = filter.getTransformMatrix();
        Eigen::MatrixXd state_x = filter.getState_();
//        std::cout << state_x.transpose().eval() << std::endl;
//        std::cout << state_x.transpose() << std::endl;
        if (uwb_data(uwb_index, 0) < imu_data(i, 0)) {
            uwb_index++;
            Eigen::Matrix<double, 1, 1> measurement_noise_matrix;
            measurement_noise_matrix.resize(1, 1);
            measurement_noise_matrix(0, 0) = 0.1;
            for (int k(1); k < uwb_data.cols(); ++k) {
                if (uwb_data(uwb_index, k) < 0.0) {
                    continue;
                }
                Eigen::Vector4d measurement_data(0, 0, 0, uwb_data(uwb_index, k));
                measurement_data.block(0, 0, 3, 1) = beacon_data.block(k - 1, 0, 1, 3).transpose();


                filter.MeasurementState(measurement_data,
                                        measurement_noise_matrix,
                                        BSE::MeasurementMethodType::NormalUwbMeasuremnt);
            }

        }


        for (int j(0); j < 3; ++j) {
            trace[j].push_back(state_x(j, 0));
            acc[j].push_back(imu_data(i, j + 1));
            gyr[j].push_back(imu_data(i, j + 4));
            velocity[j].push_back(state_x(j + 3, 0));
            attitude[j].push_back(state_x(j + 6, 0));
        }
    }

    AWF::writeVectorsToCsv<double>("./XsenseResult/trace.csv", trace);


    plt::figure();
    plt::plot(trace[0], trace[1], "-+");
    plt::title(dir_name);
    plt::grid(true);

    auto show_func = [&](vec_data d, std::string name) {
        plt::figure();
        plt::title(name);
        for (int i(0); i < d.size(); ++i) {
            plt::named_plot(std::to_string(i), d[i], "-+");
        }
        plt::grid(true);
        plt::legend();
    };


    show_func(velocity, "velocity");
    show_func(attitude, "orientation");
    show_func(acc, "acc");
    show_func(gyr, "gyr");


    plt::show();


}
