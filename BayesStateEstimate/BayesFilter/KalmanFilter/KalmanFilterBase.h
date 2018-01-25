//
// Created by steve on 18-1-25.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERBASE_H

#include "../BayesFilter.h"

namespace BSE {
    template<int StateNumber,
            int InputNumber,
            int MeasurementNumber,
            typename T>
    class KalmanFilterBase :
            public BayesFilter<StateNumber,
                    InputNumber,
                    MeasurementNumber,
                    T> {
    public:
        typedef Eigen::Matrix<T, StateNumber, StateNumber> StateTransMatrixType;
        typedef Eigen::Matrix<T, StateNumber, InputNumber> InputGainMatrixType;
        typedef Eigen::Matrix<T, MeasurementNumber, StateNumber> OutputGainMatrixType;

        typedef Eigen::Matrix<T,StateNumber,StateNumber> ProcessNoiseMatrixType;
        typedef Eigen::Matrix<T,MeasurementNumber,MeasurementNumber> MeasurementNoiseMatrixType;
        typedef Eigen::Matrix<T,StateNumber,MeasurementNumber> KMatrixType;


    protected:
        /**
         * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        StateTransMatrixType A_;
        InputGainMatrixType B_;
        OutputGainMatrixType H_;

        ProcessNoiseMatrixType Q_;
        MeasurementNoiseMatrixType R_;

        KMatrixType K_;



    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
