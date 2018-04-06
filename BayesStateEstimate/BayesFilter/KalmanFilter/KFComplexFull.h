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

            auto siuf = FullImuUpdateFunction(rbn_,
                                              time_interval_,
                                              local_g_);
            siuf.setEpsilon_(1e-1);

            auto jac_vec = siuf.derivative(state_x_,
                                           input);

            auto A = jac_vec[0];
            auto B = jac_vec[1];

            prob_state_ = A * prob_state_ * A.transpose() +
                          B * noise_matrix * B.transpose();

            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose());
            if (std::isnan(prob_state_.sum())) {
                ERROR_MSG_FLAG("porb_state_ is nan.");
            }

            state_x_ = siuf.compute(state_x_, input);
            rbn_ = Sophus::SO3d::exp(state_x_.block(6, 0, 3, 1));


        };

        /**
               * zero velocity measuremnt upd
               * @param cov_matrix
               */
        void MeasurementStateZV(Eigen::Matrix3d cov_matrix) {
            H_ = Eigen::MatrixXd::Zero(3, 15);
            H_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * 1.0;
            if (IS_DEBUG) {
                std::cout << H_ << std::endl;
                std::cout << " p * H^T :" << prob_state_ * H_.transpose().eval() << std::endl;
                std::cout << " H * P * H^T + cosv:" << H_ * prob_state_ * H_.transpose().eval() + cov_matrix
                          << std::endl;
                std::cout << "inverse " << (H_ * prob_state_ * H_.transpose().eval() + cov_matrix).inverse()
                          << std::endl;

            }

            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose().eval() + cov_matrix).inverse();
            if (std::isnan(K_.sum()) || std::isinf(K_.sum())) {
                std::cout << "K is nan" << std::endl;
            }

            /*
             * update probability
             */
            prob_state_ = (Eigen::Matrix<double, 15, 15>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = (prob_state_ + prob_state_.transpose().eval()) * 0.5;
            if (prob_state_.norm() > 10000) {
                std::cout << __FILE__
                          << ":"
                          << __LINE__
                          << " Error state prob is too big"
                          << prob_state_.norm()
                          << std::endl;
                prob_state_ /= 100.0;
            }
            if (std::isnan(prob_state_.sum()) || std::isinf(prob_state_.sum())) {
                std::cout << "state prob has nan" << std::endl;
            }

            /*
             * update state
             */
            Eigen::Vector3d m(0, 0, 0); // pseudo velocity measurement.
            dX_ = K_ * (m - state_x_.block(3, 0, 3, 1));

            state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) + dX_.block(0, 0, 6, 1);

            rbn_ = Sophus::SO3d::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = Sophus::SO3d::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            rbn_ = rbn_ * Sophus::SO3d::exp(dX_.block(6, 0, 3, 1));
            state_x_.block(6, 0, 3, 1) = rbn_.log();
            state_x_.block(9, 0, 6, 1) = state_x_.block(9, 0, 6, 1) + dX_.block(9, 0, 6, 1);


        }

        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::Matrix<double, 15, 1>
                state_x_ = Eigen::Matrix<double, 15, 1>::Zero();//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz


        Eigen::Matrix<double, 15, 15> prob_state_ = Eigen::Matrix<double, 15, 15>::Identity(); // probability of state


        Sophus::SO3d rbn_ = Sophus::SO3d::exp(
                Eigen::Vector3d(0, 0, 0));// rotation matrix from sensor frame to navigation frame

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
