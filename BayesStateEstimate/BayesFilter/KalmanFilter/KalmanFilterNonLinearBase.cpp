//
// Created by steve on 18-1-26.
//

#include "KalmanFilterNonLinearBase.h"

namespace BSE {
    bool KalmanFilterNonLinearBase::StateTransaction(const InputType &input) {
        try {
            if (StateTransactionEquation != nullptr) {
                StateTransactionEquation->(A_, B_, state_, input);
                state_probability_ = A_ * state_probability_ * A_.transpose() + Q_;
                return true;
            } else {
                return false;
//                throw std::exception().what() = "StateTransactionEquation is nullptr";
            }

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


    bool KalmanFilterNonLinearBase::MeasurementState(const MeasurementType &m) {
        try {
            if (MeasurementEquation != nullptr) {
                MeasurementEquation->(H_, state_, m, dX_);

                K_ = state_probability_ * H_.transpose() *
                     (H_ * state_probability_ * H_.transpose()).inverse();

                state_probability_ = (StateProbabilityType::Identity() - (K_ * H_))
                                     * state_probability_;
                return true;

            } else {
                return false;
//                throw std::exception().what() = "MeasurementEquation is nullptr";
            }


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


};