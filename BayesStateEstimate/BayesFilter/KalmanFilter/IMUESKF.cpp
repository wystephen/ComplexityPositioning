//
// Created by steve on 6/14/19.
//

#include "IMUESKF.h"

IMUESKF::IMUESKF(Eigen::Vector3d pos,
                 Eigen::Quaterniond qua,
                 Eigen::Vector3d velocity) : pos_(pos),
                                             qua_(qua), vel_(velocity) {


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
