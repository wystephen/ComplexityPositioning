//
// Created by steve on 18-1-25.
//

#include "KalmanFilterBase.h"

namespace BSE {
    bool KalmanFilterBase::StateTransaction(const InputType &input) {

        try {
            state_ = A_ * state_ + B_ * input;
            state_probability_ = A_ * state_probability_ * A_.transpose() + Q_;
            return true;
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

    bool KalmanFilterBase::MeasurementState(const MeasurementType &m) {
        try {
            K_ = state_probability_ * H_.transpose() *
                 (H_ * state_probability_ * H_.transpose()).inverse();
            state_ = state_ + (K_ * (m - H_ * state_));
            state_probability_ = (StateProbabilityType::Identity() - (K_ * H_))
                                 * state_probability_;
            return true;
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
}