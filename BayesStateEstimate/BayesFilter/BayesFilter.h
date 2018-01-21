//
// Created by steve on 18-1-21.
//

#ifndef COMPLEXITYPOSITIONING_BAYESFILTER_H
#define COMPLEXITYPOSITIONING_BAYESFILTER_H


template<typename StateType, typename InputType, typename MeasurementType>
class BayesFilter {
public:

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


    StateType getState_() const {
        return state_;
    }

    void setState_(StateType state_) {
        BayesFilter::state_ = state_;
    }

protected:
    StateType state_;


};


#endif //COMPLEXITYPOSITIONING_BAYESFILTER_H
