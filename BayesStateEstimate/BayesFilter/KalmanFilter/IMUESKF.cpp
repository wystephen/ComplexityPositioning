//
// Created by steve on 6/14/19.
//

#include "IMUESKF.h"

IMUESKF::IMUESKF(Eigen::Vector3d pos,
                 Eigen::Quaterniond qua,
                 Eigen::Vector3d velocity) : pos_(pos),
                                             qua_(qua),
                                             vel_(velocity) {
}


bool IMUESKF::SetProbability(double pos_std, double qua_std, double vel_std, double ab_std, double gb_std) {
	if (std::isfinite(pos_std) &&
	    std::isfinite(qua_std) &&
	    std::isfinite(vel_std) &&
	    std::isfinite(ab_std) &&
	    std::isfinite(gb_std)) {
		P_.setIdentity();
		P_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * pos_std * pos_std;
		P_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * vel_std * vel_std;
		P_.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity() * qua_std * qua_std;
		P_.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity() * ab_std * ab_std;
		P_.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity() * gb_std * gb_std;

	} else {
		// for recovery part of probability matrix from error status.
		if (std::isfinite(pos_std)) {

			P_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * pos_std * pos_std;
		}
		if (std::isfinite(vel_std)) {

			P_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * vel_std * vel_std;
		}

		if (std::isfinite(qua_std)) {

			P_.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity() * qua_std * qua_std;
		}

		if (std::isfinite(ab_std)) {

			P_.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity() * ab_std * ab_std;
		}

		if (std::isfinite(gb_std)) {

			P_.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity() * gb_std * gb_std;
		}
	}
	return true;
}


bool IMUESKF::StatePropagate(const Eigen::Vector3d &acc_data, const Eigen::Matrix3d &acc_cov,
                             const Eigen::Vector3d &gyr_data, const Eigen::Vector3d &gyr_cov, double dt) {
	pos_ = pos_ + vel_ * dt + 0.5 * (qua_ *(acc_data - acc_bias_) + gravity_vec_ ) * dt * dt;
	vel_ = vel_ + (qua_ * (acc_data - acc_bias_) + gravity_vec_) * dt;
	qua_ = BSE::ImuTools::quaternion_update(qua_, gyr_data, dt);


}

bool IMUESKF::UwbMeasurement(const double &uwb_data, const Eigen::Vector3d &uwb_beacon,
                             const Eigen::Matrix<double, 1, 1> &uwb_cov) {

}


/// GETTER AND SETTER

const Eigen::Vector3d &IMUESKF::getPos() const {
	return pos_;
}

void IMUESKF::setPos(const Eigen::Vector3d &pos) {
	pos_ = pos;
}

const Eigen::Quaterniond &IMUESKF::getQua() const {
	return qua_;
}

void IMUESKF::setQua(const Eigen::Quaterniond &qua) {
	qua_ = qua;
}

const Eigen::Vector3d &IMUESKF::getVel() const {
	return vel_;
}

void IMUESKF::setVel(const Eigen::Vector3d &vel) {
	vel_ = vel;
}

const Eigen::Vector3d &IMUESKF::getAccBias() const {
	return acc_bias_;
}

void IMUESKF::setAccBias(const Eigen::Vector3d &accBias) {
	acc_bias_ = accBias;
}

const Eigen::Vector3d &IMUESKF::getGyrBias() const {
	return gyr_bias_;
}

void IMUESKF::setGyrBias(const Eigen::Vector3d &gyrBias) {
	gyr_bias_ = gyrBias;
}
