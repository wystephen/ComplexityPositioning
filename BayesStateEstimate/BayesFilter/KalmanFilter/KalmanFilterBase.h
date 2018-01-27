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
    class KalmanFilterBase :
            public BayesFilter {
    public:

        /**
         * Initial Kalman Filter
         * @param process_noise_vec process noise vector
         * @param measurement_noise_vec measurement noise vector
         * @param initial_probability_vec state probability vector
         */
        KalmanFilterBase(Eigen::Matrix<double, Eigen::Dynamic, 1> process_noise_vec,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> measurement_noise_vec,
                         Eigen::Matrix<double, Eigen::Dynamic, 1> initial_probability_vec) :
                state_(),
                Q_(),
                R_(),
                state_probability_() {


        }


        /**
         *
         * @param input
         * @return
         */
        virtual bool StateTransaction(const Eigen::MatrixXd &input);


        /**
         *  measurement state
         * @param m  measurement state.
         * @return
         */
        virtual bool MeasurementState(const Eigen::MatrixXd &m);


        /**
         * Setter and getter for A,B&C.
         */

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


#endif //COMPLEXITYPOSITIONING_KALMANFILTERBASE_H
