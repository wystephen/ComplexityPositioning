//
// Created by steve on 18-1-27.
//

#ifndef COMPLEXITYPOSITIONING_IMUWBKF_H
#define COMPLEXITYPOSITIONING_IMUWBKF_H


#include "KalmanFilterBase.h"

namespace BSE {

    /**
     * system state: x y z vx vy vz wx wy wz
     * @tparam UWBNumber
     * @tparam time_interval
     */
    class IMUWBKFBase :
            public KalmanFilterBase {
    public:
        IMUWBKFBase(const Eigen::MatrixXd &process_noise_vec,
                    const Eigen::MatrixXd &measurement_noise_vec,
                    const Eigen::MatrixXd &initial_probability_vec) :
                KalmanFilterBase(
                        process_noise_vec, measurement_noise_vec, initial_probability_vec) {
            /**
             * define state transaction equation
             */
            StateTransactionEquationMap.insert(0, ([&](Eigen::MatrixXd &state,
                                                       Eigen::MatrixXd &state_prob,
                                                       Eigen::MatrixXd &input,
                                                       Eigen::MatrixXd &cov_input) {


                return;
            }));


            MeasurementEquationMap.insert(0, ([&](
                    Eigen::MatrixXd &state,
                    Eigen::MatrixXd &state_prob,
                    Eigen::MatrixXd &m,
                    Eigen::MatrixXd &cov_m,
                    Eigen::MatrixXd &dx

            ) {

                return;
            }));


        }

        bool initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pos = Eigen::Vector3d(0, 0, 0)
        ) {
            long double f_u(0.0), f_v(0.0), f_w(0.0);
            f_u = imu_data.col(0).mean();
            f_v = imu_data.col(1).mean();
            f_w = imu_data.col(2).mean();

            double roll = std::atan(f_v / f_w);
            double pitch = -std::asin(f_u /
                                      std::sqrt(f_u * f_u + f_v * f_v + f_w * f_w));


        }

    protected:
        double time_interval_ = 0.05;


    };

}


#endif //COMPLEXITYPOSITIONING_IMUWBKF_H
