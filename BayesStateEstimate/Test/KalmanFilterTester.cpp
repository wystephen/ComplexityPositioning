//
// Created by steve on 18-1-24.
//


#include <iostream>
#include <fstream>


#include <thread>

#include "AWF.h"


int main(int argc, char *argv[]) {
    // parameters
    std::string dir_name = "/home/steve/Data/FusingLocationData/0017/";



    // load data
    AWF::FileReader left_foot_file(dir_name + "LEFT_FOOT.data"),
            right_foot_file(dir_name + "RIGHT_FOOT.data"),
            head_imu_file(dir_name + "HEAD.data"),
            uwb_file(dir_name + "uwb_result.csv"),
            beacon_set_file(dir_name + "beaconSet.csv");

    auto left_imu_data = left_foot_file.extractDoulbeMatrix(",");
    auto right_imu_data = right_foot_file.extractDoulbeMatrix(",");
    auto head_imu_data = head_imu_file.extractDoulbeMatrix(",");
    auto uwb_data = uwb_file.extractDoulbeMatrix(",");
    auto beacon_set_data = uwb_file.extractDoulbeMatrix(",");


    //process



}