//
// Created by steve on 18-1-27.
//

#ifndef COMPLEXITYPOSITIONING_IMUWBKF_H
#define COMPLEXITYPOSITIONING_IMUWBKF_H


#include "KalmanFilterNonLinearBase.h"

namespace BSE {

    /**
     * system state: x y z vx vy vz wx wy wz
     * @tparam UWBNumber
     * @tparam time_interval
     */
    template<int UWBNumber, double time_interval>
    class IMUWBKFBase :
            public KalmanFilterNonLinearBase<9, 6, UWBNumber, double> {
    public:
        IMUWBKFBase(const Eigen::Matrix<double, 6, 1> &process_noise_vec,
                    const Eigen::Matrix<double, UWBNumber, 1> &measurement_noise_vec,
                    const Eigen::Matrix<double, 9, 1> &initial_probability_vec) :
                KalmanFilterNonLinearBase(
                        process_noise_vec, measurement_noise_vec, initial_probability_vec) {


        }

        bool initial_state(Eigen::MatrixXd data) {
            long double f_u(0.0), f_v(0.0), f_w(0.0);
            f_u = data.col(0).mean();
            f_v = data.col(1).mean();
            f_w = data.col(2).mean();

            double roll = std::atan(f_v / f_w);
            double pitch = -std::asin(f_u /
                                      std::sqrt(f_u * f_u + f_v * f_v + f_w * f_w));



        }

    protected:
        double time_interval_ = 0.05;


    };

}


#endif //COMPLEXITYPOSITIONING_IMUWBKF_H
