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
// Created by steve on 18-4-7.
//

#ifndef COMPLEXITYPOSITIONING_UKFCOMPLEX_H
#define COMPLEXITYPOSITIONING_UKFCOMPLEX_H


#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <AuxiliaryTool/GravityOrientationFunction.h>
#include <AuxiliaryTool/ImuUpdateFunction.h>
#include <AuxiliaryTool/SimpleImuUpdateFunction.h>
#include <AuxiliaryTool/MagMeasurementFunction.h>
#include <AuxiliaryTool/MagGravityMeasurementFunction.h>
#include <AuxiliaryTool/ImuTools.h>
#include <AuxiliaryTool/UwbMeasurementFunction.h>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.h"
#include "KFComplex.h"

#include "omp.h"

namespace BSE {
	class UKFComplex : public KFComplex {
		/**
		 * 15-state UKF for IMU
		 */
		UKFComplex(Eigen::Matrix<double, 15, 15> init_prob) :
				KFComplex(init_prob.block(0, 0, 9, 9)) {
			prob_state_ = init_prob;
			state_x_.setZero();
			class_name_ = "UKFComplexFull";

		};

		void initial_state(Eigen::MatrixXd imu_data,
		                   double initial_ori = 0.0,
		                   Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {
			KFComplex::initial_state(imu_data, initial_ori, initial_pose);
			state_x_.block(9, 0, 6, 1).setZero();
			std::cout << "UKFComplex initialized " << std::endl;

		}

		/**
		 * state transaction function.
		 * @param input
		 * @param noise_matrix
		 * @return
		 */
		Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
		                                           Eigen::Matrix<double, 6, 6> noise_matrix) {



		};

	};


}

#endif //COMPLEXITYPOSITIONING_UKFCOMPLEX_H
