//
// Created by steve on 18-1-26.
//

#ifndef COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
#define COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H

#include <iostream>
#include <functional>
#include <map>

#include <Eigen/Dense>

#include "../BayesFilter.h"
#include "../BayesFilter.cpp"

namespace BSE {
    class KalmanFilterBase :
            public BayesFilter {
    public:


        KalmanFilterBase(const Eigen::MatrixXd &process_noise_vec,
                         const Eigen::MatrixXd &measurement_noise_vec,
                         const Eigen::MatrixXd &initial_probability_vec) :
                BayesFilter() {

        }


        /**
         *
         * @param input
         * @param cov_input
         * @return
         */
        bool StateTransaction(const Eigen::MatrixXd &input,
                              const Eigen::MatrixXd &cov_input);

        /**
         *  state transaction...
         * @param input
         * @param cov_input
         * @param methodType
         * @return
         */
        bool StateTransaction(const Eigen::MatrixXd &input,
                              const Eigen::MatrixXd &cov_input,
                              int methodType = 0);


        bool MeasurementState(const Eigen::MatrixXd &m,
                              const Eigen::MatrixXd &cov_m);

        /**
         *  measurement State function
         * @param m
         * @param cov_m
         * @param methodType
         * @return
         */
        bool MeasurementState(const Eigen::MatrixXd &m,
                              const Eigen::MatrixXd &cov_m,
                              int methodType = 0);


    protected:
        std::map<int, std::function<void(Eigen::MatrixXd &,//state
                                         Eigen::MatrixXd &,//state probability
                                         const Eigen::MatrixXd &,//input (included input data and auxiliary data)
                                         const Eigen::MatrixXd &// cov input
        )> > StateTransactionEquationMap = {};

        std::map<int, std::function<void(Eigen::MatrixXd &,//state
                                         Eigen::MatrixXd &,//state probability
                                         Eigen::MatrixXd &,//measurement
                                         Eigen::MatrixXd &,// cov measurement
                                         Eigen::MatrixXd &// dx
        )>> MeasurementEquationMap = {};
        /**
         * Setter and getter for A,B&C.
         */
    public:


    protected:

        /**
         * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        Eigen::MatrixXd A_ = Eigen::MatrixXd();
        Eigen::MatrixXd B_ = Eigen::MatrixXd();
        Eigen::MatrixXd H_ = Eigen::MatrixXd();
        Eigen::MatrixXd Q_ = Eigen::MatrixXd();
        Eigen::MatrixXd R_ = Eigen::MatrixXd();
        Eigen::MatrixXd K_ = Eigen::MatrixXd();
        Eigen::MatrixXd dX_ = Eigen::MatrixXd();

    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
