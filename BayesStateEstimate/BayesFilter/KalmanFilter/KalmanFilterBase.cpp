//
// Created by steve on 18-1-25.
//

#include "KalmanFilterBase.h"

namespace BSE {
    bool KalmanFilterBase::StateTransaction(const Eigen::MatrixXd &input) {

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

    bool KalmanFilterBase::MeasurementState(const Eigen::MatrixXd &m) {
        try {

            dX_ = m - H_ * state_;
            state_ = state_ + (K_ * (m - H_ * state_));

            K_ = state_probability_ * H_.transpose() *
                 (H_ * state_probability_ * H_.transpose()).inverse();

            state_probability_ =
                    (Eigen::MatrixXd::Identity(state_probability_.rows(), state_probability_.cols()) - (K_ * H_))
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

    const Eigen::MatrixXd &KalmanFilterBase::getA_() const {
        return A_;
    }

    void KalmanFilterBase::setA_(const Eigen::MatrixXd &A_) {
        KalmanFilterBase::A_ = A_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getB_() const {
        return B_;
    }

    void KalmanFilterBase::setB_(const Eigen::MatrixXd &B_) {
        KalmanFilterBase::B_ = B_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getH_() const {
        return H_;
    }

    void KalmanFilterBase::setH_(const Eigen::MatrixXd &H_) {
        KalmanFilterBase::H_ = H_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getQ_() const {
        return Q_;
    }

    void KalmanFilterBase::setQ_(const Eigen::MatrixXd &Q_) {
        KalmanFilterBase::Q_ = Q_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getR_() const {
        return R_;
    }

    void KalmanFilterBase::setR_(const Eigen::MatrixXd &R_) {
        KalmanFilterBase::R_ = R_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getK_() const {
        return K_;
    }

    void KalmanFilterBase::setK_(const Eigen::MatrixXd &K_) {
        KalmanFilterBase::K_ = K_;
    }

    const Eigen::MatrixXd &KalmanFilterBase::getDX_() const {
        return dX_;
    }

    void KalmanFilterBase::setDX_(const Eigen::MatrixXd &dX_) {
        KalmanFilterBase::dX_ = dX_;
    }

}
