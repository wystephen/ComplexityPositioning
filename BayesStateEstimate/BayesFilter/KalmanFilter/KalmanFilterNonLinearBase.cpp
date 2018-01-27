//
// Created by steve on 18-1-26.
//

#include "KalmanFilterNonLinearBase.h"

namespace BSE {
    KalmanFilterNonLinearBase::StateTransaction(const Eigen::MatrixXd &input, const Eigen::MatrixXd &cov_input) {
        StateTransaction(input, 0);
    }

    bool KalmanFilterNonLinearBase::StateTransaction(const Eigen::MatrixXd &input, const Eigen::MatrixXd &cov_input,int methodType) {
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


    bool KalmanFilterNonLinearBase::MeasurementState(const Eigen::MatrixXd &m, int methodType) {
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
                std::cout <<"}"<< std::endl;
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

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getA_() const {
        return A_;
    }

    void KalmanFilterNonLinearBase::setA_(const Eigen::MatrixXd &A_) {
        KalmanFilterNonLinearBase::A_ = A_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getB_() const {
        return B_;
    }

    void KalmanFilterNonLinearBase::setB_(const Eigen::MatrixXd &B_) {
        KalmanFilterNonLinearBase::B_ = B_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getH_() const {
        return H_;
    }

    void KalmanFilterNonLinearBase::setH_(const Eigen::MatrixXd &H_) {
        KalmanFilterNonLinearBase::H_ = H_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getQ_() const {
        return Q_;
    }

    void KalmanFilterNonLinearBase::setQ_(const Eigen::MatrixXd &Q_) {
        KalmanFilterNonLinearBase::Q_ = Q_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getR_() const {
        return R_;
    }

    void KalmanFilterNonLinearBase::setR_(const Eigen::MatrixXd &R_) {
        KalmanFilterNonLinearBase::R_ = R_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getK_() const {
        return K_;
    }

    void KalmanFilterNonLinearBase::setK_(const Eigen::MatrixXd &K_) {
        KalmanFilterNonLinearBase::K_ = K_;
    }

    const Eigen::MatrixXd &KalmanFilterNonLinearBase::getDX_() const {
        return dX_;
    }

    void KalmanFilterNonLinearBase::setDX_(const Eigen::MatrixXd &dX_) {
        KalmanFilterNonLinearBase::dX_ = dX_;
    }


};