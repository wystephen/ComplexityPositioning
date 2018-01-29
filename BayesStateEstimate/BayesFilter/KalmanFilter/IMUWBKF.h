//
// Created by steve on 18-1-27.
//

#ifndef COMPLEXITYPOSITIONING_IMUWBKF_H
#define COMPLEXITYPOSITIONING_IMUWBKF_H

#include <sophus/so3.h>
#include <sophus/se3.h>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.cpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace BSE {

    /**
     * system state: x y z vx vy vz wx wy wz
     * @tparam UWBNumber
     * @tparam time_interval
     */
    class IMUWBKFBase :
            public KalmanFilterBase {
    public:
        IMUWBKFBase(
                const Eigen::MatrixXd &initial_probability_vec) :
                KalmanFilterBase(
                        initial_probability_vec) {



            /**
             * define state transaction equation
             */
            StateTransactionEquationMap.insert({0, ([](Eigen::MatrixXd &state,
                                                       Eigen::MatrixXd &state_prob,
                                                       const Eigen::MatrixXd &input,
                                                       const Eigen::MatrixXd &cov_input) {


                return;
            })});


            MeasurementEquationMap.insert({0, ([](
                    Eigen::MatrixXd &state,
                    Eigen::MatrixXd &state_prob,
                    const Eigen::MatrixXd &m,
                    const Eigen::MatrixXd &cov_m,
                    Eigen::MatrixXd &dx

            ) {

                return;
            })});


        }

        /**
         * initial own pose.
         * @param imu_data
         * @param initial_ori
         * @param initial_pos
         * @return
         */
        bool initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pos = Eigen::Vector3d(0, 0, 0)
        ) {
            long double f_u(0.0), f_v(0.0), f_w(0.0);
//            f_u = imu_data.col(0).mean();
//            f_v = imu_data.col(1).mean();
//            f_w = imu_data.col(2).mean();
            Eigen::Vector3d acc = imu_data.block(0, 0, imu_data.rows(), 3).colwise().mean();
            double g = acc.norm();

            double roll = std::atan(f_v / f_w);
            double pitch = -std::asin(f_u /
                                      std::sqrt(f_u * f_u + f_v * f_v + f_w * f_w));

            double min_roll(0.0), min_pitch(0.0);
            double error = 10000.0;

            auto g_error = [&g, &acc](double roll, double pitch, double yaw) -> double {

                auto rotate_matrix = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
//                std::cout << typeid(rotate_matrix) << std::endl;
                return std::abs(g - (rotate_matrix * acc)(2));
            };
            auto ge(0.0);
            for (double tr(-M_PI); tr < M_PI; tr += 0.2 / 180.0 * M_PI) {
                for (double tp(-M_PI); tp < M_PI; tp += 0.2 / 180.0 * M_PI) {

                    ge = g_error(tr, tp, initial_ori);
                    if (ge < error) {
                        error = ge;
                        min_roll = tr;
                        min_pitch = tp;

                    }
                }
            }
//            std::cout << state_
            state_.block(0, 0, 3, 1) = initial_pos;
            state_.block(3, 0, 3, 1).setZero();
            state_.block(6, 0, 3, 1) = Eigen::Vector3d(min_roll, min_pitch, initial_ori);

            auto rotate_matrix = (Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX())
                                 * Eigen::AngleAxisd(state_(7), Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()));
            std::cout << "value angle:" << state_.block(6,0,3,1).transpose() << std::endl;
            std::cout << "eular angle:" << rotate_matrix.toRotationMatrix().eulerAngles(0,1,2).transpose() << std::endl;
            std::cout << "before acc:" << acc.transpose() << std::endl;
            std::cout << "after acc:" << (rotate_matrix * acc).transpose() << std::endl;


        }

        /**
         * Auxiliary function convert euler angle to rotation matrix.
         * @param pitch
         * @param roll
         * @param yaw
         * @return
         */
        inline Eigen::Matrix3d Ang2RotMatrix(double pitch, double roll, double yaw) {
            double cp = std::cos(pitch);
            double sp = std::sin(pitch);

            double cr = std::cos(roll);
            double sr = std::sin(roll);

            double cy = std::cos(yaw);
            double sy = std::sin(yaw);

            Eigen::Matrix3d C;
            C << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy + sr * sy),
                    cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
                    -sp, sr * cp, cr * cp;
            return C;
        }


    protected:
        double time_interval_ = 0.05;


    };

}


#endif //COMPLEXITYPOSITIONING_IMUWBKF_H
