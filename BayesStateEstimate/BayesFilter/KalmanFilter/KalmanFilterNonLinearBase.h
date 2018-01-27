//
// Created by steve on 18-1-26.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H

#include <iostream>
#include <functional>
#include <map>
#include "KalmanFilterBase.h"

namespace BSE {
    class KalmanFilterNonLinearBase :
            public KalmanFilterBase {
    public:


        KalmanFilterNonLinearBase(const Eigen::MatrixXd &process_noise_vec,
                                  const Eigen::MatrixXd &measurement_noise_vec,
                                  const Eigen::MatrixXd &initial_probability_vec)
                : KalmanFilterBase(process_noise_vec, measurement_noise_vec, initial_probability_vec) {}


        bool StateTransaction(const Eigen::MatrixXd &input);

        /**
         * state transaction function using the state transaction equation in StateTransactonMap
         * @param input
         * @param methodType
         * @return
         */
        bool StateTransaction(const Eigen::MatrixXd &input, int methodType = 0);


        bool MeasurementState(const Eigen::MatrixXd &m);

        /**
         * choice measurement equation based on method Type.
         * @param m
         * @param methodType
         * @return
         */
        bool MeasurementState(const Eigen::MatrixXd &m, int methodType = 0);


    protected:
        std::map<int, std::function<void(Eigen::MatrixXd &,
                                         Eigen::MatrixXd &,
                                         Eigen::MatrixXd &,
                                         Eigen::MatrixXd &)> *> StateTransactionEquationMap = {};

        std::map<int, std::function<void(Eigen::MatrixXd &,
                                         Eigen::MatrixXd &,
                                         Eigen::MatrixXd &,
                                         Eigen::MatrixXd &)> *> MeasurementEquationMap = {};


    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
