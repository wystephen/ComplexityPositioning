//
// Created by steve on 18-1-26.
//

#include "KalmanFilterBase.h"

namespace BSE {
    bool KalmanFilterBase::StateTransaction(const Eigen::MatrixXd &input,
                                                     const Eigen::MatrixXd &cov_input) {
        return StateTransaction(input, cov_input, 0);

    }

    bool KalmanFilterBase::StateTransaction(const Eigen::MatrixXd &input,
                                                     const Eigen::MatrixXd &cov_input,
                                                     int methodType) {
        try {
            if (StateTransactionEquationMap.count(methodType) > 0) {
                StateTransactionEquationMap[methodType]->(A_, B_, state_, input);
                state_probability_ = A_ * state_probability_ * A_.transpose() + Q_;
                return true;
            } else {
                std::cout << "method type [" << methodType
                          << "] is invalid. \nFollowing methodType contained in the map";
                for (auto val:StateTransactionEquationMap) {
                    std::cout << val.first << " ";
                }
                std::cout << std::endl;
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

    bool KalmanFilterBase::MeasurementState(const Eigen::MatrixXd &m,
                                                     const Eigen::MatrixXd &cov_m) {
        return MeasurementState(m, cov_m, 0);
    }

    bool KalmanFilterBase::MeasurementState(const Eigen::MatrixXd &m,
                                                     const Eigen::Matrix &cov_m,
                                                     int methodType = 0) {
        try {
            if (MeasurementEquationMap.count(methodType) > 0) {
                MeasurementEquationMap[methodType]->(H_, state_, m, dX_);

                K_ = state_probability_ * H_.transpose() *
                     (H_ * state_probability_ * H_.transpose()).inverse();

                state_probability_ =
                        (Eigen::MatrixXd::Identity(state_probability_.rows(), state_probability_.cols()) - (K_ * H_))
                        * state_probability_;
                return true;

            } else {
                std::cout << "method type [" << methodType
                          << "] is invalid. \nFollowing methodType contained in the map {";
                for (auto val:MeasurementEquationMap) {
                    std::cout << val.first << ",";
                }
                std::cout << "}" << std::endl;
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