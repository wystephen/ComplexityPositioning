//
// Created by steve on 18-1-26.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H

#include <iostream>
#include <functional>
#include "KalmanFilterBase.h"

namespace BSE {
    template<int StateNumber,
            int InputNumber,
            int MeasurementNumber,
            typename T>
    class KalmanFilterNonLinearBase :
            public KalmanFilterBase<
                    StateNumber,
                    InputNumber,
                    MeasurementNumber,
                    T> {
    public:


        KalmanFilterNonLinearBase(const Eigen::Matrix<double, InputNumber, 1> &process_noise_vec,
                                  const Eigen::Matrix<double, MeasurementNumber, 1> &measurement_noise_vec,
                                  const Eigen::Matrix<double, StateNumber, 1> &initial_probability_vec)
                : KalmanFilterBase(process_noise_vec, measurement_noise_vec, initial_probability_vec) {}

    protected:
        std::function<void(decltype(A_) & ,
                           decltype(B_) & ,
                           decltype(state_) & ,
                           decltype(input_) & )> *StateTransactionEquation = nullptr;

        std::function<void(decltype(H_) & ,
                           decltype(state_) & ,
                           decltype(m_) & )> *MeasurementEquation = nullptr;


//        std::function<bool(StateTransMatrixType ,
//                           InputGainMatrixType,
//                           StateType,
//                           InputType)> stateTransactionEquation;



    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
