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
// Created by steve on 18-3-18.
//

#include <iostream>




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

    std::string dir_name = "/home/steve/Data/XsensUwb/MTI700/0003/";

    AWF::FileReader imu_file(dir_name + "imu.data");
    AWF::FileReader uwb_file(dir_name + "uwb_data.csv");
    AWF::FileReader beacon_file(dir_name + "beaconset_no_mac.csv");


    Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
    Eigen::MatrixXd beacon_data = beacon_file.extractDoulbeMatrix(",");

//    uwb_data.block(0, 0, uwb_data.rows(), 1) =
//            uwb_data.block(0, 0, uwb_data.rows(), 1) - double(uwb_data(0, 0) + imu_data(0, 0));
    double time_offset = double(uwb_data(0, 0) - imu_data(0, 0));
    for (int i(0); i < uwb_data.rows(); ++i) {
        uwb_data(i, 0) = uwb_data(i, 0) - time_offset;
    }
    auto uwb_tool = BSE::UwbTools(uwb_data,
                                  beacon_data);

    Eigen::MatrixXd optimize_trace = uwb_tool.uwb_position_function();

    std::vector<std::vector<double>> optimize_trace_vec = {{},
                                                           {},
                                                           {}};
    for (int i(0); i < optimize_trace.rows(); ++i) {
        for (int j(0); j < 3; ++j) {
            optimize_trace_vec[j].push_back(optimize_trace(i, j));
        }
    }


    plt::figure();
    plt::title("uwb trace");
    plt::plot(optimize_trace_vec[0], optimize_trace_vec[1], "-*");
    plt::legend();
    plt::grid(true);
//    plt::title(dir_name);
//    plt::grid(true);





    plt::show();


}