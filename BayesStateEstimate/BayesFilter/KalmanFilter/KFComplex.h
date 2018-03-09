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
            auto g = acc.norm();
//            local_g_ = g;


            auto g_error = [&, &g, acc](double roll, double pitch, double yaw) -> double {

                auto rotate_matrix = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
                return std::abs(std::abs(g) * local_g_ / std::abs(local_g_) + (rotate_matrix * acc)(2));
            };
            auto ge(0.0);
//

            /**
             * find initial euler angle through optimization.
             */
            double tr = 0.0;
            double tp = 0.0;

            double step_len = 0.000005;
            double update_rate = 0.5;
            int iter_counter = 0;
            double current_error(g_error(tr, tp, initial_ori));
            while (current_error > 1e-7 && iter_counter < 30000) {

                iter_counter++;
//                tr +=(g_error)
                // compute gradient
                double delta_tr = (g_error(tr + step_len, tp, initial_ori) - current_error) / step_len;
                double delta_tp = (g_error(tr, tp + step_len, initial_ori) - current_error) / step_len;
                if (std::isnan(delta_tp) || std::isnan(delta_tr)) {
                    delta_tp = 0.0001;
                    delta_tr = 0.0001;

                    continue;
                }
                // update state.
                tr -= delta_tr * update_rate;
                tp -= delta_tp * update_rate;

                while (tr > M_PI + 0.01) {
                    tr -= 2.0 * M_PI;
                }
                while (tr < -M_PI - 0.01) {

                    tr += 2.0 * M_PI;
                }

                while (tp > M_PI + 0.01) {
                    tp -= 2.0 * M_PI;
                }

                while (tp < -M_PI - 0.01) {
                    tp += 2.0 * M_PI;
                }
                /*
                 * Reduce the learning rate.
                 */
                if (update_rate > 0.00001) {
                    update_rate *= 0.99;
                }


                current_error = g_error(tr, tp, initial_ori);
                if (IS_DEBUG) {
                    std::cout << iter_counter
                              << ":"
                              << current_error
                              << "{"
                              << tr
                              << ":"
                              << tp
                              << "}"
                              << std::endl;
                }


            }


            state_x_.block(0, 0, 3, 1) = initial_pose;
            state_x_.block(3, 0, 3, 1).setZero();
            state_x_.block(6, 0, 3, 1) = Eigen::Vector3d(tr, tp, initial_ori);

            rotation_q_ = (Eigen::AngleAxisd(tr, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(tp, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(initial_ori, Eigen::Vector3d::UnitZ()));

            std::cout << "complex value angle:" << state_x_.block(6, 0, 3, 1).transpose() << std::endl;
            std::cout << "complex eular angle:" << rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2).transpose()
                      << std::endl;
            std::cout << "complex before acc:" << acc.transpose() << std::endl;
            std::cout << "complex after acc:" << (rotation_q_ * acc).transpose() << std::endl;

        }

        Eigen::Matrix<double, 9, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
                                                  Eigen::Matrix<double, 6, 6> noise_matrix) {

            Eigen::Vector3d acc(input.block(0, 0, 3, 1));
            Eigen::Vector3d gyr(input.block(0, 0, 3, 1));

            if (IS_DEBUG) {
                std::cout << "acc in navigation frame:"
                          << (rotation_q_ * acc).transpose()
                          << std::endl;
            }

            if (gyr.norm() > 1e-8) {
                Eigen::Quaterniond tmp_q =
                        Eigen::AngleAxisd(gyr(0), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(gyr(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(gyr(2), Eigen::Vector3d::UnitZ());

                rotation_q_ = rotation_q_ * tmp_q;

                rotation_q_.normalize();

            }


            Eigen::Vector3d gravity_g(0, 0, local_g_);
            Eigen::Vector3d linear_acc = rotation_q_.toRotationMatrix() * acc + gravity_g;
            if (IS_DEBUG) {
                std::cout << "acc in navigation frame:" << (rotation_q_ * acc).transpose();
                std::cout << "\nlinear_acc:" << linear_acc.transpose() << std::endl;
            }

            auto converted_input = input;
            converted_input.block(0, 0, 3, 1) = linear_acc;
            converted_input.block(3, 0, 3, 1) = gyr;


            Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(9, 9);
            // x y z
            A_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            A_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * time_interval_;
            // vx vy vz
            A_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();
            // wx wy wz
            A_.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

            Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(9, 6);


            B_.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * time_interval_;
            B_.block(6, 3, 3, 3) = Eigen::Matrix3d::Identity() * time_interval_;


            state_x_ = A_ * state_x_ + B_ * converted_input;
            state_x_.block(6, 0, 3, 1) = rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2);

            if (IS_DEBUG) {
                std::cout << "state trans P:"
                          << prob_state_ << std::endl;
                std::cout << " A * P * A^ T "
                          << A_ * prob_state_ * A_.transpose() << std::endl;
                std::cout << "B * cove * B.transpose() "
                          << B_ * converted_input * B_.transpose() << std::endl;
            }
            // unconverted value
//                         B_.block(6, 0, 3, 3) = rotate_q_.toRotationMatrix() ;//* time_interval_;
            prob_state_ = A_ * prob_state_ * A_.transpose() +
                          B_ * noise_matrix * B_.transpose();
            if (std::isnan(prob_state_.sum())) {
                std::cout << "state prob is naa: " << prob_state_ << std::endl;
            }


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
//                         rotation_m = (2.0 * Eigen::Matrix3d::Identity() + omega) *
//                                      (2.0 * Eigen::Matrix3d::Identity() - omega).inverse()
//                                      * rotation_m;
            rotation_m = (Eigen::Matrix3d::Identity() - omega) * rotation_m;

//                         rotate_q_ = delta_q.inverse() * rotate_q_;
            rotation_q_ = Eigen::Quaterniond(rotation_m);
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



        double local_g_ = -9.81; // local gravity acc.


        bool IS_DEBUG = false; // debug flag.




    };
}


#endif //COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
