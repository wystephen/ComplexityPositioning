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

namespace BSE {
    class KalmanFilterNonLinearBase :
            public BayesFilter {
    public:


        KalmanFilterNonLinearBase(const Eigen::MatrixXd &process_noise_vec,
                                  const Eigen::MatrixXd &measurement_noise_vec,
                                  const Eigen::MatrixXd &initial_probability_vec) : BayesFilter() ,
        Q_(Eigen::MatrixXd::Identity(process_noise_vec.rows(),process_noise_vec.rows())*process_noise_vec){

        }


        bool StateTransaction(const Eigen::MatrixXd &input);

        /**
         * state transaction function using the state transaction equation in StateTransactonMap
         * @param input
         * @param methodType
         * @return
         */
        bool StateTransaction(const Eigen::MatrixXd &input,
                              const Eigen::MatrixXd &cov_input,
                              int methodType = 0);


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
        /**
         * Setter and getter for A,B&C.
         */
    public:
        const Eigen::MatrixXd &getA_() const;

        void setA_(const Eigen::MatrixXd &A_);

        const Eigen::MatrixXd &getB_() const;

        void setB_(const Eigen::MatrixXd &B_);

        const Eigen::MatrixXd &getH_() const;

        void setH_(const Eigen::MatrixXd &H_);

        const Eigen::MatrixXd &getQ_() const;

        void setQ_(const Eigen::MatrixXd &Q_);

        const Eigen::MatrixXd &getR_() const;

        void setR_(const Eigen::MatrixXd &R_);

        const Eigen::MatrixXd &getK_() const;

        void setK_(const Eigen::MatrixXd &K_);

        const Eigen::MatrixXd &getDX_() const;

        void setDX_(const Eigen::MatrixXd &dX_);

    protected:

        /**
         * X_i=A*X_{i-1}+B*u_i+w_i
         * z_i=H*X_i+v_i
         * w_i \in Q
         * v_i \in R
         */
        Eigen::MatrixXd A_;


        Eigen::MatrixXd B_;
        Eigen::MatrixXd H_;

        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;

        Eigen::MatrixXd K_;

        Eigen::MatrixXd dX_;

    };
}


#endif //COMPLEXITYPOSITIONING_KALMANFILTERNONLINEARBASE_H
