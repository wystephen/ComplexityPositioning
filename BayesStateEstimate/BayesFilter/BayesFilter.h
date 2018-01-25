//
// Created by steve on 18-1-21.
//

#ifndef COMPLEXITYPOSITIONING_BAYESFILTER_H
#define COMPLEXITYPOSITIONING_BAYESFILTER_H

#include <Eigen/Dense>

namespace BSE {
    template<int StateNum,
            int InputNum,
            int MeasurementNum,
            typename T>
    class BayesFilter {
    public:
        typedef Eigen::Matrix<T, StateNum, 1> StateType;
        typedef Eigen::Matrix<T, StateNum, StateNum> StateProbabilityType;
        typedef Eigen::Matrix<T, InputNum, 1> InputType;
        typedef Eigen::Matrix<T, MeasurementNum, 1> MeasurementType;



        /**
         *
         */
        virtual void initial(){
        }

        /**
         * use state transaction equation
         * @param input
         * @return
         */
        virtual bool StateTransaction(const InputType& input);

        /**
         * use measurement equation update the probability and value of system state.
         * @param m
         * @return
         */
        virtual bool MeasurementState(const MeasurementType& m);

        /**
         * iterator use the measurement equation.
         * @param m
         * @param times times of use @MeasurementState
         * @return
         */
        virtual bool MeasurementState(const MeasurementType& m, int times){
            for(int i(0);i<times;++i){
                MeasurementState(m);
            }
        }


        /**
         * get system state
         * @return
         */
        StateType getState_() const {
            return state_;
        }

        /**
         * set system state.
         * @param state_
         */
        void setState_(StateType state_) {
            BayesFilter::state_ = state_;
        }

        /**
         *
         * @return
         */
        const StateProbabilityType &getState_probability_() const {
            return state_probability_;
        }

        /**
         *
         * @param state_probability_
         */
        void setState_probability_(const StateProbabilityType &state_probability_) {
            BayesFilter::state_probability_ = state_probability_;
        }

    protected:
        StateType state_;
        StateProbabilityType state_probability_;


    };

}


#endif //COMPLEXITYPOSITIONING_BAYESFILTER_H
