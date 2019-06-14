//
// Created by steve on 6/14/19.
//

#ifndef COMPLEXITYPOSITIONING_IMUESKF_H
#define COMPLEXITYPOSITIONING_IMUESKF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


// TODO: achieve using Template achieve runnable in embed device without huge modify.
class IMUESKF {
public:
	/**
	 * @brief Initial the class setting pos, orientation and initial velocity(quite important).
	 * @param pos
	 * @param qua
	 * @param velocity
	 */
	IMUESKF(Eigen::Vector3d pos, Eigen::Quaterniond qua, Eigen::Vector3d velocity);

	bool StatePropagate(Eigen::Vector3d acc_data,
	                    Eigen::Vector3d acc_cov,
	                    Eigen::Vector3d gyr_data,
	                    Eigen::Vector3d gyr_cov,
	                    double dt);

	bool UwbMeasurement(double uwb_data, Eigen::Vector3d uwb_beacon,
	                    Eigen::Matrix<double, 1, 1> uwb_cov);


private:
	Eigen::Vector3d pos_;

	Eigen::Quaterniond qua_;
	Eigen::Vector3d vel_;
	Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d gyr_bias_ = Eigen::Vector3d::Zero();

	Eigen::Matrix<double, 15, 1> error_state_ = Eigen::Matrix<double, 15, 1>::Zero();//  Error state
	// pos, vel, angle, acc_bias, gyr_bias

	Eigen::Matrix<double, 15, 15> P_;// probability of error state

	const Eigen::Matrix<double, 15, 15> I15_ = Eigen::Matrix<double, 15, 15>::Identity();

public:
	const Eigen::Vector3d &getPos() const;

	void setPos(const Eigen::Vector3d &pos);

	const Eigen::Quaterniond &getQua() const;

	void setQua(const Eigen::Quaterniond &qua);

	const Eigen::Vector3d &getVel() const;

	void setVel(const Eigen::Vector3d &vel);

	const Eigen::Vector3d &getAccBias() const;

	void setAccBias(const Eigen::Vector3d &accBias);

	const Eigen::Vector3d &getGyrBias() const;

	void setGyrBias(const Eigen::Vector3d &gyrBias);
};


#endif //COMPLEXITYPOSITIONING_IMUESKF_H
