//
// Created by steve on 18-1-21.
//

#ifndef COMPLEXITYPOSITIONING_BAYESFILTER_H
#define COMPLEXITYPOSITIONING_BAYESFILTER_H

#include <Eigen/Dense>

namespace BSE {
    template<int StateNum, int InputNumber, int MeasurementNumber,typename T>
    class BayesFilter {
    public:
        typedef Eigen::Matrix<T,StateNum,1> StateType;
        typedef Eigen::Matrix<T,InputNumber,1> InputType;
        typedef Eigen::Matrix<T,MeasurementNumber,1> MeasurementType;

        /**
         * use state transaction equation
         * @param input
         * @return
         */
        virtual bool StateTransaction(InputType input);

        /**
         * use measurement equation update the probability and value of system state.
         * @param m
         * @return
         */
        virtual bool MeasurementState(MeasurementType m);

        /**
         * iterator use the measurement equation.
         * @param m
         * @param times times of use @MeasurementState
         * @return
         */
        virtual bool MeasurementState(MeasurementType m, int times);


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

    protected:
        StateType state_;




    };

}


#endif //COMPLEXITYPOSITIONING_BAYESFILTER_H
