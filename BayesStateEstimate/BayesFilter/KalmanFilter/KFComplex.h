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

#ifndef COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
#define COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H


#include <sophus/so3.h>
#include <sophus/se3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <AuxiliaryTool/GravityOrientationFunction.h>
#include <AuxiliaryTool/ImuUpdateFunction.h>
#include <AuxiliaryTool/SimpleImuUpdateFunction.h>
#include <AuxiliaryTool/MagMeasurementFunction.h>
#include <AuxiliaryTool/MagGravityMeasurementFunction.h>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.h"

namespace BSE {


    class KFComplex {
    public:

        KFComplex(Eigen::Matrix<double, 9, 9> init_prob) {
            prob_state_ = init_prob;
            state_x_.setZero();
        }


        bool initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {
            long double f_u(0.0), f_v(0.0), f_w(0.0);
            Eigen::Vector3d acc = imu_data.block(0, 0, imu_data.rows(), 3).colwise().mean();
            Eigen::Vector3d mag = imu_data.block(0, 6, imu_data.rows(), 3).colwise().mean();
            auto g = acc.norm();
            local_g_ = -1.0 * g;

            auto gof = GravityOrientationFunction(acc, -1.0 * local_g_ / std::abs(local_g_) * acc.norm(), initial_ori);

            auto res_vec = gof.minimize_error(Eigen::Vector2d(0.0, 0.0),1000,0.01);
            double tr = res_vec[0](0);
            double tp = res_vec[0](1);

            auto f = [](double a) -> double {
                while (a > M_PI + 1e-3) {
                    a -= 2.0 * M_PI;
                }
                while (a < M_PI - 1e-3) {
                    a += 2.0 * M_PI;
                }
                return a;
            };
            tr = f(tr);
            tp = f(tp);


            state_x_.block(0, 0, 3, 1) = initial_pose;
            state_x_.block(3, 0, 3, 1).setZero();
            state_x_.block(6, 0, 3, 1) = Eigen::Vector3d(tr, tp, initial_ori);

            rotation_q_ = (Eigen::AngleAxisd(tr, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(tp, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(initial_ori, Eigen::Vector3d::UnitZ()));

//            mag_func.mag_nav_ = rotation_q_ * (mag/mag.norm());
            auto acc_nav = rotation_q_ * acc;
            auto mag_nav = rotation_q_ * mag;
            std::cout << "acc nav:"
                      << acc_nav.transpose()
                      << " mag nava:"
                      << mag_nav.transpose()
                      << std::endl;
            mag_func.setMag_nav(rotation_q_ * mag);
            mg_fuc.setMag_nav(rotation_q_ * mag);
            mg_fuc.setGravity_nav_(rotation_q_ * acc);



            std::cout << "complex value angle:" << state_x_.block(6, 0, 3, 1).transpose()
                      << std::endl;
            std::cout << "complex eular angle:" << rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2).transpose()
                      << std::endl;
            std::cout << "complex before acc:" << acc.transpose() << std::endl;
            std::cout << "complex after acc:" << (rotation_q_ * acc).transpose() << std::endl;
            std::cout << "complex after mg func:" << mg_fuc.compute(state_x_).transpose() << std::endl;

        }

        /**
         * State Transaction Function. Update state based on imu measurement.
         * @param input
         * @param noise_matrix
         * @return
         */
        Eigen::Matrix<double, 9, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
                                                  Eigen::Matrix<double, 6, 6> noise_matrix) {

            auto siuf = SimpleImuUpdateFunction(rotation_q_, time_interval_, local_g_);
            auto jac_vec = siuf.derivative(state_x_, input);
            auto A = jac_vec[0];
            auto B = jac_vec[1];

            prob_state_ = A * prob_state_ * A.transpose() +
                          B * noise_matrix * B.transpose();

            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose());
            if (std::isnan(prob_state_.sum())) {
                ERROR_MSG_FLAG("porb_state_ is nan.");
            }

            state_x_ = siuf.compute(state_x_, input);
            rotation_q_ = Eigen::AngleAxisd(state_x_(6, 0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(state_x_(7, 0), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(state_x_(8, 0), Eigen::Vector3d::UnitZ());


            return state_x_;
        };


        void MeasurementStateZV(Eigen::Matrix3d cov_matrix) {
            H_ = Eigen::MatrixXd::Zero(3, 9);
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
                std::cout << "K is nana" << std::endl;
            }

            /*
             * update probability
             */
            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = (prob_state_ + prob_state_.transpose().eval()) * 0.5;
            if (prob_state_.norm() > 10000) {
                std::cout << __FILE__
                          << ":"
                          << __LINE__
                          << " Error state prob is too big"
                          << std::endl;
                prob_state_ /= 100.0;
            }
            if (std::isnan(prob_state_.sum()) || std::isinf(prob_state_.sum())) {
                std::cout << "state prob has nan" << std::endl;
            }

            /*
             * update state
             */
            Eigen::Vector3d m(0, 0, 0);
            Eigen::MatrixXd tdx = K_ * (m - state_x_.block(3, 0, 3, 1));

            state_x_ += tdx;


            Eigen::Quaterniond delta_q = Eigen::AngleAxisd(tdx(6), Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(tdx(7), Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(tdx(8), Eigen::Vector3d::UnitZ());
            if (std::isnan(state_x_.sum())) {
                std::cout << "some error " << std::endl;
            }

            Eigen::Matrix3d rotation_m(rotation_q_.toRotationMatrix());
            Eigen::Matrix3d omega = Eigen::Matrix3d::Zero();
            omega << 0.0, tdx(8), -tdx(7),
                    -tdx(8), 0.0, tdx(6),
                    tdx(7), -tdx(6), 0.0;
            omega *= -1.0;
            rotation_m = (Eigen::Matrix3d::Identity() - omega) * rotation_m;

            rotation_q_ = Eigen::Quaterniond(rotation_m);
            return;

        }


        /**
         * correcting orientation.
         * @param input
         */
        void MeasurementAngleCorrect(Eigen::Matrix<double, 3, 1> input,
                                     Eigen::Matrix<double, 3, 3> cov_m) {
            Eigen::Vector3d tmp_mag = input;
            rotation_q_.normalize();
            state_x_.block(6, 0, 3, 1) = rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2);

            auto t_vec = mag_func.derivative(state_x_);
            H_ = t_vec[0];
            ////TODO: correct this.
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

            dX_ = K_ * (input - mag_func.compute(state_x_));
            std::cout << " diff: " << (input - mag_func.compute(state_x_)).transpose();

            state_x_ += dX_;


            Eigen::Matrix3d rotation_m(rotation_q_.toRotationMatrix());
            Eigen::Matrix3d omega = Eigen::Matrix3d::Zero();
            omega << 0.0, dX_(8), -dX_(7),
                    -dX_(8), 0.0, dX_(6),
                    dX_(7), -dX_(6), 0.0;
            omega *= -1.0;
//                         rotation_m = (2.0 * Eigen::Matrix3d::Identity() + omega) *
//                                      (2.0 * Eigen::Matrix3d::Identity() - omega).inverse()
//                                      * rotation_m;
            rotation_m = (Eigen::Matrix3d::Identity() - omega) * rotation_m;

//                         rotate_q_ = delta_q.inverse() * rotate_q_;
            rotation_q_ = Eigen::Quaterniond(rotation_m);
            rotation_q_.normalize();
            state_x_.block(6, 0, 3, 1) = rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2);
            std::cout << "input:"
                      << input.transpose()
                      << "reverted input:"
                      << (rotation_q_ * input).transpose()
                      << std::endl;

            return;


        }


        /**
         * correcting orientation.
         * @param input acc and mag
         */
        void MeasurementAngleCorrectMG(Eigen::Matrix<double, 6, 1> input,
                                       Eigen::Matrix<double, 6, 6> cov_m) {
            Eigen::Vector3d tmp_mag = input.block(3, 0, 3, 1);
            Eigen::Vector3d tmp_acc = input.block(0, 0, 3, 1);
            Eigen::Matrix<double, 6, 1> g_and_mag;
            g_and_mag.block(0, 0, 3, 1) = tmp_acc;// / tmp_acc.norm();
            g_and_mag.block(3, 0, 3, 1) = tmp_mag;// / tmp_mag.norm();


            rotation_q_.normalize();
            state_x_.block(6, 0, 3, 1) = rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2);

            auto t_vec = mg_fuc.derivative(state_x_);
            H_ = t_vec[0];
            ////TODO: correct this.
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
//            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

            dX_ = K_ * (g_and_mag - mg_fuc.compute(state_x_));
//            std::cout << "diff: "
//                      << (g_and_mag - mg_fuc.compute(state_x_)).transpose()
//                      << std::endl;
//            std::cout << "gmag:"
//                      << g_and_mag.transpose()
//                      << std::endl;
//            std::cout << "fuc :"
//                      << mg_fuc.compute(state_x_).transpose()
//                      << std::endl;

            state_x_ += dX_;


//            std::cout << "dx:"
//                      << dX_.transpose()
//                      << std::endl;

            /*---------------------------------------*/
            /////////
            Eigen::Quaterniond tmp_q = Eigen::AngleAxisd(dX_(6), Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(dX_(7), Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(dX_(8), Eigen::Vector3d::UnitZ());
//            tmp_q.normalize();
            rotation_q_ = tmp_q * rotation_q_;
//            rotation_q_ = rotation_q_ * tmp_q;

            ////////
//            Eigen::Matrix3d rotation_m(rotation_q_.toRotationMatrix());
//            Eigen::Matrix3d omega = Eigen::Matrix3d::Zero();
//            omega << 0.0, dX_(8), -dX_(7),
//                    -dX_(8), 0.0, dX_(6),
//                    dX_(7), -dX_(6), 0.0;
//            omega *= -1.0;
//                         rotation_m = (2.0 * Eigen::Matrix3d::Identity() + omega) *
//                                      (2.0 * Eigen::Matrix3d::Identity() - omega).inverse()
//                                      * rotation_m;
//            rotation_m = (Eigen::Matrix3d::Identity() - omega) * rotation_m;

//            rotation_q_ = Eigen::Quaterniond(rotation_m);

            /*-00000000000000000000000000000000000000*/
            rotation_q_.normalize();
            state_x_.block(6, 0, 3, 1) = rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2);

//            std::cout << "input:"
//                      << input.transpose()
//            std::cout << "reve:"
//                      << (rotation_q_ * tmp_acc).transpose()
//                      << std::endl;

            return;


        }


        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::Matrix<double, 9, 1> state_x_ = Eigen::Matrix<double, 9, 1>::Zero();//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz

        Eigen::Quaterniond rotation_q_ = Eigen::Quaterniond::Identity();


        Eigen::Matrix<double, 9, 9> prob_state_ = Eigen::Matrix<double, 9, 9>::Identity(); // probability of state



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


#endif //COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
