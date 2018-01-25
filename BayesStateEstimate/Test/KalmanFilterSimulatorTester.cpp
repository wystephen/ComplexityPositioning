//
// Created by steve on 18-1-24.
//

#include <iostream>
#include <fstream>


#include <thread>

#include "AWF.h"

#include <Eigen/Dense>

int main(){

    auto imu_reader = AWF::FileReader("./test/imu.csv");
    auto ground_truth_reader =AWF::FileReader("./test/groundtruth.csv");

    auto imu_data = imu_reader.extractDoulbeMatrix(",");
    auto g_trace = ground_truth_reader.extractDoulbeMatrix(",");

    std::cout << imu_data << std::endl;

    std::cout << g_trace << std::endl;


}