//
// Created by steve on 18-1-21.
//

#ifndef COMPLEXITYPOSITIONING_BAYESFILTER_H
#define COMPLEXITYPOSITIONING_BAYESFILTER_H

#include <Eigen/Dense>

namespace BSE {
    class BayesFilter {
    public:

        /**
         *
         */
        virtual void initial() {
        }

        /**
         * use state transaction equation
         * @param input
         * @return
         */
        virtual bool StateTransaction(const Eigen::MatrixXd &input,
                                      const Eigen::MatrixXd &input_cov)= 0;

        /**
         * use measurement equation update the probability and value of system state.
         * @param m
         * @return
         */
        virtual bool MeasurementState(const Eigen::MatrixXd &m,
                                      const Eigen::MatrixXd &cov_m)=0;


//// Might be define as iterator the measurement minimize the linearization error



        /**
         * get system state
         * @return
         */
        Eigen::MatrixXd getState_() const {
            return state_;
        }

        /**
         * set system state.
         * @param state_
         */
        void setState_(Eigen::MatrixXd state_) {
            BayesFilter::state_ = state_;
        }

        /**
         *
         * @return
         */
        const Eigen::MatrixXd &getState_probability_() const {
            return state_probability_;
        }

        /**
         *
         * @param state_probability_
         */
        void setState_probability_(const Eigen::MatrixXd &state_probability_) {
            BayesFilter::state_probability_ = state_probability_;
        }

        bool IS_DEBUG = false;

    protected:
        Eigen::MatrixXd state_;
        Eigen::MatrixXd state_probability_;
        Eigen::MatrixXd input_;
        Eigen::MatrixXd m_;


    };

}


#endif //COMPLEXITYPOSITIONING_BAYESFILTER_H
