//
// Created by steve on 18-1-25.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERBASE_H

#include <unsupported/Eigen/AutoDiff>
#include <iostream>
#include "../BayesFilter.h"
#include "../BayesFilter.cpp"

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
        using StateTransMatrixType=Eigen::Matrix<T, StateNumber, StateNumber>;
        using InputGainMatrixType=Eigen::Matrix<T, StateNumber, InputNumber>;
        using OutputGainMatrixType=Eigen::Matrix<T, MeasurementNumber, StateNumber>;

        using ProcessNoiseMatrixType=Eigen::Matrix<T, StateNumber, StateNumber>;
        using MeasurementNoiseMatrixType=Eigen::Matrix<T, MeasurementNumber, MeasurementNumber>;
        using KMatrixType=Eigen::Matrix<T, StateNumber, MeasurementNumber>;

        /**
         * Initial Kalman Filter
         * @param process_noise_vec process noise vector
         * @param measurement_noise_vec measurement noise vector
         * @param initial_probability_vec state probability vector
         */
        KalmanFilterBase(Eigen::Matrix<double, InputNumber, 1> process_noise_vec,
                         Eigen::Matrix<double, MeasurementNumber, 1> measurement_noise_vec,
                         Eigen::Matrix<double, StateNumber, 1> initial_probability_vec) :
                state_(StateType::Identity()),
                Q_(ProcessNoiseMatrixType::Identity() * process_noise_vec),
                R_(MeasurementNoiseMatrixType::Identity() * measurement_noise_vec),
                state_probability_(StateProbabilityType::Identity() * initial_probability_vec) {


        }


        /**
         *
         * @param input
         * @return
         */
        virtual bool StateTransaction(const decltype(input_) &input);


        /**
         *  measurement state
         * @param m  measurement state.
         * @return
         */
        virtual bool MeasurementState(const decltype(m_) &m);


        /**
         * Setter and getter for A,B&C.
         */
        const StateTransMatrixType &getA_() const;

        void setA_(const StateTransMatrixType &A_);

        const InputGainMatrixType &getB_() const;

        void setB_(const InputGainMatrixType &B_);

        const OutputGainMatrixType &getH_() const;

        void setH_(const OutputGainMatrixType &H_);

    protected:
        /**
         * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        StateTransMatrixType A_ = StateTransMatrixType::Identity();


    protected:
        InputGainMatrixType B_ = InputGainMatrixType::Identity();
        OutputGainMatrixType H_ = OutputGainMatrixType::Identity();

        ProcessNoiseMatrixType Q_;
        MeasurementNoiseMatrixType R_;

        KMatrixType K_ = KMatrixType::Identity();

        decltype(state_) dX_;


    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
