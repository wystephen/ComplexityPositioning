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

//    std::string file_name = "/home/steve/Data/XsensData/line-high.csv";
    std::string file_name = "/home/steve/Data/XsensData/line-low.csv";
//    std::string file_name = "/home/steve/Data/XsensData/two round high.csv";
//    std::string file_name = "/home/steve/Data/XsensData/mav_data.csv";

    AWF::FileReader imu_file(file_name);

    Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");

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
    filter.setTime_interval_(0.005);
    filter.initial_state(imu_data.block(0, 1, 50, 6), 0.0);
    filter.setLocal_g_(9.81);
//    filter.IS_DEBUG = true;


    auto imu_tool = BSE::ImuTools();

    std::vector<std::vector<double>> trace = {{},
                                              {},
                                              {}};


    typedef std::vector<std::vector<double>> vec_data;
    vec_data acc = {{},{},{}};
    vec_data gyr = {{},{},{}};
    vec_data velocity = {{},{},{}};
    vec_data attitude = {{},{},{}};




    for (int i(0); i < imu_data.rows(); ++i) {
        filter.StateTransaction(imu_data.block(i, 1, 1, 6).transpose(),
                                process_noise_matrix,
                                BSE::StateTransactionMethodType::NormalRotation);

        auto state_T = filter.getTransformMatrix();
        Eigen::MatrixXd state_x = filter.getState_();
//        std::cout << state_x.transpose().eval() << std::endl;
//        std::cout << state_x.transpose() << std::endl;


        for (int j(0); j < 3; ++j) {
            trace[j].push_back(state_T(j,3));
            acc[j].push_back(imu_data(i,j+1));
            gyr[j].push_back(imu_data(i,j+4));
            velocity[j].push_back(state_x(j+3,0));
            attitude[j].push_back(state_x(j+6,0));
        }
    }

    AWF::writeVectorsToCsv<double>("./XsenseResult/trace.csv",trace);


    plt::figure();
    plt::plot(trace[0], trace[1], "-+");
    plt::title(file_name);
    plt::grid(true);

    auto show_func = [&](vec_data d, std::string name){
        plt::figure();
        plt::title(name);
        for(int i(0);i<d.size();++i){
            plt::named_plot(std::to_string(i),d[i],"-+");
        }
        plt::grid(true);
        plt::legend();
    };


    show_func(velocity,"velocity");
    show_func(attitude,"orientation");
    show_func(acc,"acc");
    show_func(gyr,"gyr");



    plt::show();


}
