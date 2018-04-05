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
// Created by steve on 18-4-3.
//

#ifndef COMPLEXITYPOSITIONING_KFCOMPLEXFULL_H
#define COMPLEXITYPOSITIONING_KFCOMPLEXFULL_H

#include <sophus/so3.hpp>
#include <sophus/se3.h>

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

namespace BSE {

    class KFComplexFull : public KFComplex {
    public:
        KFComplexFull(Eigen::Matrix<double, 15, 15> init_prob) :
                KFComplex(init_prob.block(0, 0, 9, 9)) {
            prob_state_ = init_prob;
            state_x_.setZero();
            class_name_ = "KFComplexFull";
        }


        void initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {
            std::cout << "complex full initial." << std::endl;
            KFComplex::initial_state(imu_data, initial_ori, initial_pose);
            state_x_.block(9, 0, 6, 1).setZero();
        }

        Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
                                                   Eigen::Matrix<double, 6, 6> noise_matrix) {




        };


        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::Matrix<double, 15, 1>
                state_x_ = Eigen::Matrix<double, 15, 1>::Zero();//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz


        Eigen::Matrix<double, 15, 15> prob_state_ = Eigen::Matrix<double, 15, 15>::Identity(); // probability of state


        Sophus::SO3 rbn_ = Sophus::SO3(0, 0, 0);// rotation matrix from sensor frame to navigation frame

        /**
        * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        Eigen::MatrixXd A_ = Eigen::MatrixXd();
        Eigen::MatrixXd B_ = Eigen::MatrixXd();
        Eigen::MatrixXd H_ = Eigen::MatrixXd();
        Eigen::MatrixXd Q_ = Eigen::MatrixXd();
        Eigen::MatrixXd R_ = Eigen::MatrixXd();
        Eigen::MatrixXd K_ = Eigen::MatrixXd();
        Eigen::MatrixXd dX_ = Eigen::MatrixXd();


        double time_interval_ = 0.005;// time interval

        MagMeasurementFunction mag_func = MagMeasurementFunction();
        MagGravityMeasurementFunction mg_fuc = MagGravityMeasurementFunction();


        double local_g_ = -9.81; // local gravity acc.


        bool IS_DEBUG = false; // debug flag.





    };
}


#endif //COMPLEXITYPOSITIONING_KFCOMPLEXFULL_H
