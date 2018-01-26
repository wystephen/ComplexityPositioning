//
// Created by steve on 18-1-27.
//

#ifndef COMPLEXITYPOSITIONING_IMUWBKF_H
#define COMPLEXITYPOSITIONING_IMUWBKF_H


#include "KalmanFilterNonLinearBase.h"

namespace BSE {

    template<int UWBNumber, double time_interval>
    class IMUWBKF :
            public KalmanFilterNonLinearBase<9, 6, UWBNumber, double> {
    public:
        IMUWBKF(const Eigen::Matrix<double, 6, 1> &process_noise_vec,
                const Eigen::Matrix<double, UWBNumber, 1> &measurement_noise_vec,
                const Eigen::Matrix<double, 9, 1> &initial_probability_vec) : KalmanFilterNonLinearBase(
                process_noise_vec, measurement_noise_vec, initial_probability_vec) {

        }


    };

}


#endif //COMPLEXITYPOSITIONING_IMUWBKF_H
