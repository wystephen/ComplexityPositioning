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

        KFComplex(Eigen::Matrix<double, 15, 15> init_prob) {
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

        Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
                                                   Eigen::Matrix<double, 6, 6> noise_matrix) {


        };


        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::Matrix<double, 15, 1> state_x_ = Eigen::Matrix<double, 15, 1>::Zero();//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz

        Eigen::Quaterniond rotation_q_ = Eigen::Quaterniond::Identity();


        Eigen::Matrix<double, 15, 15> prob_state_ = Eigen::Matrix<double, 15, 15>::Identity(); // probability of state


        double time_interval_ = 0.005;// time interval

        double local_g_ = -9.81; // local gravity acc.


        bool IS_DEBUG = false; // debug flag.




    };
}


#endif //COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
