//
// Created by steve on 18-1-25.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERBASE_H

#include <iostream>
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

        typedef Eigen::Matrix<T, StateNumber, StateNumber> ProcessNoiseMatrixType;
        typedef Eigen::Matrix<T, MeasurementNumber, MeasurementNumber> MeasurementNoiseMatrixType;
        typedef Eigen::Matrix<T, StateNumber, MeasurementNumber> KMatrixType;

//        typedef std::function<StateType(StateType,InputType)> StateTransFunc;
//        typedef std::function<MeasurementType(MeasurementType,)

        /**
         *
         * @param input
         * @return
         */
        virtual bool StateTransaction(const InputType &input) {
            try {
                state_ = A_ * state_ + B_ * input;
                state_probability_ = A_ * state_probability_ * A_.transpose() + Q_;
                return true;
            } catch (std::exception &e) {
                std::cout << __FILE__
                          << ":"
                          << __LINE__
                          << ":"
                          << __FUNCTION__
                          << ":"
                          << e.what()
                          << std::endl;
                return false;
            }
        }

        virtual bool MeasurementState(const MeasurementType &m) {
            try {
                K_ = state_probability_ * H_.transpose() *
                     (H_ * state_probability_ * H_.transpose()).inverse();
                state_ = state_ + (K_ * (m - H_ * state_));
                state_probability_ = (StateProbabilityType::Identity() - (K_ * H_))
                                     * state_probability_;
                return true;
            } catch (std::exception &e) {
                std::cout << __FILE__
                          << ":"
                          << __LINE__
                          << ":"
                          << __FUNCTION__
                          << ":"
                          << e.what()
                          << std::endl;
                return false;
            }
        }


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

        StateType dX_;


    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
