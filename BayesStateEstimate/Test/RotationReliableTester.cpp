

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
// Created by steve on 18-4-20.
//


#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <AWF.h>

#include <AuxiliaryTool/ImuTools.h>


int main() {
	double step_len = 0.1 / 180.0 * M_PI;

	//eigen quaternion test

	auto test_own_q = [&step_len](int axis_id = 0) {
		double acc_angle = 0.0;

		auto logger_ptr = AWF::AlgorithmLogger::getInstance();

		Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
		Eigen::Quaterniond qr(1.0, 0.0, 0.0, 0.0);
		logger_ptr->addPlotEvent("test_own_q" + std::to_string(axis_id), "q",
		                         q.toRotationMatrix().eulerAngles(0, 1, 2));
		logger_ptr->addPlotEvent("test_own_q" + std::to_string(axis_id), "qr",
		                         qr.toRotationMatrix().eulerAngles(0, 1, 2));
		Eigen::Vector3d angle_add(0.0, 0.0, 0.0);
		if (axis_id < 3) {
			angle_add(axis_id) = step_len;
		}
		while (acc_angle < 4 * M_PI) {
			acc_angle += step_len;
//			q = BSE::ImuTools::quaternion_update(q, angle_add, 1.0);
//			qr = BSE::ImuTools::quaternion_update(qr, angle_add, -1.0);
			q = BSE::ImuTools::quaternion_left_update(q, angle_add, 1.0);
			qr = BSE::ImuTools::quaternion_left_update(qr, angle_add, -1.0);

			logger_ptr->addPlotEvent("test_own_q" + std::to_string(axis_id), "q",
			                         q.toRotationMatrix().eulerAngles(0, 1, 2));
			logger_ptr->addPlotEvent("test_own_q" + std::to_string(axis_id), "qr",
			                         qr.toRotationMatrix().eulerAngles(0, 1, 2));

		}


	};

	for(int i(0);i<3;++i){
		test_own_q(i);
	}



	auto logger_ptr = AWF::AlgorithmLogger::getInstance();
	logger_ptr->outputAllEvent(true);

}


