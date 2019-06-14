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
	 * @param pos position[x,y,z] in [m].
	 * @param qua represent rotation in 3D world frame. L2(qua) == 1.0.
	 * @param velocity velocity[vx,vy,vz] in [m/s]
	 */
	IMUESKF(Eigen::Vector3d pos, Eigen::Quaterniond qua, Eigen::Vector3d velocity);


	/**
	 * @brief set initial probability. If *_std is NAN, such value will not change.
	 * @param pos_std std. of pose
	 * @param qua_std std. of quaternion.
	 * @param vel_std  std. of velocity.
	 * @param ab_std  std. of acc bias.
	 * @param gb_std  std. of gyr bias.
	 * @return
	 */
	bool SetProbability(double pos_std, double qua_std, double vel_std, double ab_std, double gb_std);

	bool StatePropagate(const Eigen::Vector3d &acc_data,
	                    const Eigen::Matrix3d &acc_cov,
	                    const Eigen::Vector3d &gyr_data,
	                    const Eigen::Vector3d &gyr_cov,
	                    double dt);

	bool UwbMeasurement(double uwb_data, Eigen::Vector3d uwb_beacon,
	                    Eigen::Matrix<double, 1, 1> uwb_cov);


private:
	Eigen::Vector3d pos_;

	Eigen::Quaterniond qua_;
	Eigen::Vector3d vel_;
	Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d gyr_bias_ = Eigen::Vector3d::Zero();

	///// pos, vel, angle, acc_bias, gyr_bias   /////
	Eigen::Matrix<double, 15, 1> error_state_ = Eigen::Matrix<double, 15, 1>::Zero();//  Error state

	Eigen::Matrix<double, 15, 15> P_ = Eigen::Matrix<double,15,15>::Identity();// probability of error state

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
