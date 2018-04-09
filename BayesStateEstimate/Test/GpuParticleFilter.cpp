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
// Created by steve on 18-3-7.
//

#include <iostream>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/compute/function.hpp>
#include <boost/compute/system.hpp>
#include <boost/compute/algorithm/count_if.hpp>
#include <boost/compute/container/vector.hpp>
#include <boost/compute/iterator/buffer_iterator.hpp>
#include <boost/compute/random/default_random_engine.hpp>
#include <boost/compute/types/fundamental.hpp>

#include "BayesFilter/ParticleFilter/GpuParticleFilter.h"
#include <AWF.h>
#include "../AuxiliaryTool/UwbTools.h"
#include "../AuxiliaryTool/UwbTools.cpp"

#include "../AuxiliaryTool/ImuTools.h"
#include "../AuxiliaryTool/ImuTools.cpp"


namespace compute = boost::compute;


namespace plt = matplotlibcpp;
//#ifdef _OPENACC
//#include<openacc.h>
//#endif

int main() {
//#ifdef OPENACC
//	printf("Number of device :%d\n",acc_get_num_devices(acc_device_not_host));
//#else
//	printf("OpenACC is not support.\n");
//#endif
	/// Load Data from files
	std::cout.precision(30);
	// parameters
	std::string dir_name = "/home/steve/Data/FusingLocationData/0013/";



	// load data
	AWF::FileReader imu_file(dir_name + "LEFT_FOOT.data"),
//            right_foot_file(dir_name + "RIGHT_FOOT.data"),
//            head_imu_file(dir_name + "HEAD.data"),
			uwb_file(dir_name + "uwb_result.csv"),
			beacon_set_file(dir_name + "beaconSet.csv");

	Eigen::MatrixXd imu_data = imu_file.extractDoulbeMatrix(",");
//    Eigen::MatrixXd right_imu_data = right_foot_file.extractDoulbeMatrix(",");
//    Eigen::MatrixXd head_imu_data = head_imu_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd uwb_data = uwb_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd beacon_set_data = beacon_set_file.extractDoulbeMatrix(",");

	assert(beacon_set_data.rows() == (uwb_data.cols() - 1));

//	auto imu_tool = BSE::ImuTools();

	BSE::ImuTools::processImuData(imu_data);


//    for()

	auto gpu_pf = BSE::GpuParticleFilter<float, 6>(1000, Eigen::Matrix<float, 1, 6>::Zero());


}