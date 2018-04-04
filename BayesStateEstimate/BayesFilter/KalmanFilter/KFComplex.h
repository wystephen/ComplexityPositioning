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
#include <AuxiliaryTool/ImuTools.h>
#include <AuxiliaryTool/UwbMeasurementFunction.h>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.h"

namespace BSE {


    class KFComplex {
    public:

        KFComplex(Eigen::Matrix<double, 9, 9> init_prob) {
            prob_state_ = init_prob;
            state_x_.setZero();
        }


        /**
         * Initial filter.
         * @param imu_data : source data of IMU when it in static state.
         * @param initial_ori : orientation in x-o-y plane.
         * @param initial_pose : initial pose(3-axis)
         * @return
         */
        bool initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {
            long double f_u(0.0), f_v(0.0), f_w(0.0);
            Eigen::Vector3d acc = imu_data.block(0, 0, imu_data.rows(), 3).colwise().mean();
            Eigen::Vector3d mag = imu_data.block(0, 6, imu_data.rows(), 3).colwise().mean();
            auto g = acc.norm();
            local_g_ = -1.0 * g;

            auto gof = GravityOrientationFunction(acc, -1.0 * local_g_ / std::abs(local_g_) * acc.norm(), initial_ori);

            auto res_vec = gof.minimize_error(Eigen::Vector2d(0.0, 0.0), 1000, 0.01);
            double tr = res_vec[0](0);
            double tp = res_vec[0](1);


            state_x_.block(0, 0, 3, 1) = initial_pose;
            state_x_.block(3, 0, 3, 1).setZero();

            rbn_ = Sophus::SO3::exp(Eigen::Vector3d(tr, tp, initial_ori));
            state_x_.block(6, 0, 3, 1) = rbn_.log();

            auto acc_nav = rbn_.matrix() * acc;
            auto mag_nav = rbn_.matrix() * mag;
            std::cout << "acc nav:"
                      << acc_nav.transpose()
                      << " mag nava:"
                      << mag_nav.transpose()
                      << std::endl;
            mag_func.setMag_nav(rbn_.matrix() * mag);
            mag_func.setEpsilon_(1e-8);
            mg_fuc.setEpsilon_(1e-6);
            mg_fuc.setMag_nav(rbn_.matrix() * mag);
            mg_fuc.setGravity_nav_(Eigen::Vector3d(0, 0, 1.0));


            std::cout << "complex value angle:" << state_x_.block(6, 0, 3, 1).transpose()
                      << std::endl;
            std::cout << "complex before acc:" << acc.transpose() << std::endl;
            std::cout << "complex after acc:" << (rbn_ * acc).transpose() << std::endl;
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

            auto siuf = SimpleImuUpdateFunction(rbn_,
                                                time_interval_,
                                                local_g_);
            siuf.setEpsilon_(1e-7);

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
//            rbn_ = siuf.rbn;
            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
//            rotation_q_ = Eigen::AngleAxisd(state_x_(6, 0), Eigen::Vector3d::UnitX()) *
//                          Eigen::AngleAxisd(state_x_(7, 0), Eigen::Vector3d::UnitY()) *
//                          Eigen::AngleAxisd(state_x_(8, 0), Eigen::Vector3d::UnitZ());


            return state_x_;
        };


        /**
         * zero velocity measuremnt upd
         * @param cov_matrix
         */
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
                std::cout << "K is nan" << std::endl;
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

            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
            rbn_ = Sophus::SO3::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            state_x_.block(6, 0, 3, 1) = rbn_.log();


        }


        /**
         * correcting orientation.
         * @param input
         */
        void MeasurementAngleCorrect(Eigen::Matrix<double, 3, 1> input,
                                     Eigen::Matrix<double, 3, 3> cov_m) {
            Eigen::Vector3d tmp_mag = input;
            input = input / double(input.norm());


            auto t_vec = mag_func.derivative(state_x_);
            H_ = t_vec[0];
            ////TODO: correct this.
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

            dX_ = K_ * (input - mag_func.compute(state_x_));
//            std::cout << " standard: " << (mag_func.compute(state_x_)).transpose();

            state_x_.block(0, 0, 6, 1) += dX_.block(0, 0, 6, 1);
            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = rbn_ * Sophus::SO3::exp(dX_.block(6, 0, 3, 1));
            rbn_ = Sophus::SO3::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            state_x_.block(6, 0, 3, 1) = rbn_.log();


//            std::cout << "input:"
//                      << input.transpose()
//                      << "reverted input:"
//                      << (rbn_.matrix() * input).transpose()
//                      << "world:"
//                      << mag_func.mag_nav_.transpose()
//                      << std::endl;
            auto logger_ptr = AWF::AlgorithmLogger::getInstance();
            logger_ptr->addPlotEvent(class_name_+"angle_correct", "input", input);
            logger_ptr->addPlotEvent(class_name_+"angle_correct", "reverted", Eigen::Vector3d(rbn_.matrix() * input));
            logger_ptr->addPlotEvent(class_name_+"angle_correct", "world_value", mag_func.mag_nav_);
            logger_ptr->addPlotEvent(class_name_+"probability", "P", prob_state_);

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
            g_and_mag.block(0, 0, 3, 1) = tmp_acc / tmp_acc.norm();
            g_and_mag.block(3, 0, 3, 1) = tmp_mag / tmp_mag.norm();

            mg_fuc.setEpsilon_(1e-2 * M_PI / 180.0);
            auto t_vec = mg_fuc.derivative(state_x_);
            H_ = t_vec[0];
            ////TODO: correct this.
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

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
            auto t = mg_fuc.compute(state_x_);


            state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) +
                                         dX_.block(0, 0, 6, 1);


            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = rbn_ * Sophus::SO3::exp(dX_.block(6, 0, 3, 1));
            rbn_ = Sophus::SO3::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            state_x_.block(6, 0, 3, 1) = rbn_.log();

            auto logger_ptr = AWF::AlgorithmLogger::getInstance();
            logger_ptr->addPlotEvent(class_name_+"gravity", "before_acc", g_and_mag.block(0, 0, 3, 1));
            logger_ptr->addPlotEvent(class_name_+"gravity", "converted_acc", rbn_.matrix() * g_and_mag.block(0, 0, 3, 1));
            logger_ptr->addPlotEvent(class_name_+"gravity", "nav_acc", mg_fuc.gravity_nav_);


            return;
        }


        /**
         *  measurement according to uwb.
         * @param measurement
         */
        void MeasurementUwb(Eigen::Matrix<double, 4, 1> input,
                            Eigen::Matrix<double, 1, 1> cov_m) {
            auto uwbFunc = UwbMeasurementFunction(input.block(0, 0, 3, 1));
            uwbFunc.setEpsilon_(1e-8);

            auto d_vec = uwbFunc.derivate(state_x_);
            H_ = d_vec[0];//1x9 matrix
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

            Eigen::Matrix<double, 1, 1> t_m;
            t_m(0, 0) = input(3);
            dX_ = K_ * (t_m - uwbFunc.compute(state_x_));

            auto t = mg_fuc.compute(state_x_);


            state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) +
                                         dX_.block(0, 0, 6, 1);


            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = rbn_ * Sophus::SO3::exp(dX_.block(6, 0, 3, 1));
            rbn_ = Sophus::SO3::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            state_x_.block(6, 0, 3, 1) = rbn_.log();
        }


        /**
         * Use all the uwb measurement in each epoch to correct the system state.
         * @param input
         * @param cov_m
         */
        void MeasurementUwbFull(Eigen::MatrixXd input,
                                Eigen::MatrixXd cov_m) {

            assert(input.cols() == 4);
            assert(input.rows() == cov_m.rows());
//            std::cout <<"input:"<< input << std::endl;
//            std::cout << "cov m:" << cov_m << std::endl;

            auto logger_ptr = AWF::AlgorithmLogger::getInstance();

            H_ = Eigen::MatrixXd(input.rows(), state_x_.rows());

            auto y = Eigen::MatrixXd(input.rows(), 1);

            for (int i(0); i < input.rows(); ++i) {
                auto uwbFunc = UwbMeasurementFunction(input.block(i, 0, 1, 3).transpose());
                uwbFunc.setEpsilon_(1e-8);

                H_.block(i, 0, 1, H_.cols()) = uwbFunc.derivate(state_x_)[0];
                y(i, 0) = uwbFunc.compute(state_x_)(0);
            }

//            logger_ptr->addPlotEvent("uwb_measurement", "src", input.block(0, 3, input.rows(), 1));
//            logger_ptr->addPlotEvent("uwb_measurement", "y", y);
            Eigen::MatrixXd tmd(1,1);
            tmd(0,0) = (input.block(0, 3, input.rows(), 1) - y).norm();
            logger_ptr->addPlotEvent("xsense_uwb", class_name_+"diff", tmd);


//            std::cout << "H:" << H_ << std::endl;
//            std::cout << "y:" << y << std::endl;
            K_ = (prob_state_ * H_.transpose().eval()) *
                 (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

            prob_state_ = (Eigen::Matrix<double, 9, 9>::Identity() - K_ * H_) * prob_state_;
            prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose().eval());

            dX_ = K_ * (input.block(0, 3, input.rows(), 1) - y);

            auto t = mg_fuc.compute(state_x_);


            state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) +
                                         dX_.block(0, 0, 6, 1);


            rbn_ = Sophus::SO3::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = rbn_ * Sophus::SO3::exp(dX_.block(6, 0, 3, 1));
            rbn_ = Sophus::SO3::exp(dX_.block(6, 0, 3, 1)) * rbn_;
            state_x_.block(6, 0, 3, 1) = rbn_.log();
        }

        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::MatrixXd state_x_ = Eigen::Matrix<double, 9, 1>::Zero();//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz



        Eigen::MatrixXd prob_state_ = Eigen::Matrix<double, 9, 9>::Identity(); // probability of state



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


        Sophus::SO3 rbn_ = Sophus::SO3(0, 0, 0);// rotation matrix from sensor frame to navigation frame

        std::string class_name_= "KFComplex";


    };
}


#endif //COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
