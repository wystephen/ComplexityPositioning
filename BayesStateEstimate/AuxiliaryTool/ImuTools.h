/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 18-2-3.
//

#ifndef COMPLEXITYPOSITIONING_IMUTOOLS_H
#define COMPLEXITYPOSITIONING_IMUTOOLS_H


#include <iostream>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace BSE {
	namespace ImuTools {

		/**
		 * zero velocity detector based on GLRT algorithm.
		 * @param u
		 * @return
		 */
		bool GLRT_Detector(Eigen::MatrixXd u) {
			if (u.cols() == 6 && u.rows() != 6) {
				Eigen::MatrixXd tu = u * 1.0;
//        u = u.transpose();
				u = tu.transpose();
			}
			assert(u.rows() == 6 || "u must be a 6 rows matrix(each col represent acc and gyro at one moement");
			Eigen::Vector3d ya_m;
			double g = 9.8;
			double sigma_a_ = 0.05;
			double sigma_g_ = 0.05 * M_PI / 180.0;
			double ZeroDetectorWindowSize_ = u.rows();
			double gamma_ = 240;

			double T(0.0);

			for (int i(0); i < 3; ++i) {
				ya_m(i) = u.block(i, 0, 1, u.cols()).mean();
			}

			Eigen::Vector3d tmp;

			for (int i(0); i < u.cols(); ++i) {

				tmp = u.block(0, i, 3, 1) - g * ya_m / ya_m.norm();
				if (std::isnan(tmp.sum())) {
					std::cout << "nan at tmp in " << __FUNCTION__ << ":"
					          << __FILE__ << ":" << __LINE__ << std::endl;
				}


				T += (u.block(3, i, 3, 1).transpose() * u.block(3, i, 3, 1) / sigma_g_ +
				      tmp.transpose() * tmp / sigma_a_).sum();


			}


			T = T / double(ZeroDetectorWindowSize_);

			if (T < gamma_) {
				return true;

			} else {

				return false;
			}
		}

		/**
		*  process the imu data according to the typical sensor model
		* @param imu_data after preprocess time[s] acc_(x,y,z)[m*s^-2] gyr_(x,y,z)[rad*s^-1] mag_(x,y,z) pressure
		 */
		void processImuData(Eigen::MatrixXd &imu_data) {
			Eigen::MatrixXd tmp_data(imu_data);

			int row(imu_data.rows());
			int col(imu_data.cols());
			imu_data.resize(row, 1 + 3 + 3 + 3 + 1);//
			// time
			imu_data.block(0, 0, row, 1) = tmp_data.block(0, 1, row, 1) * 1.0;
			imu_data.block(0, 1, row, 3) = tmp_data.block(0, 2, row, 3) * 9.81;
			imu_data.block(0, 4, row, 3) = tmp_data.block(0, 5, row, 3) * (M_PI / 180.0);
			imu_data.block(0, 7, row, 3) = tmp_data.block(0, 8, row, 3) * 1.0;
			return;
		}

		/**
		 * Convert eular angle in three axis as quaternion.
		 * @param ang
		 * @return
		 */
		Eigen::Quaterniond angle2q(const Eigen::Vector3d &ang) {
			Eigen::Quaterniond q = Eigen::AngleAxisd(ang(0), Eigen::Vector3d::UnitX()) *
			                       Eigen::AngleAxisd(ang(1), Eigen::Vector3d::UnitY()) *
			                       Eigen::AngleAxisd(ang(2), Eigen::Vector3d::UnitZ());

//            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
//            double bank = ang(0);
//            double attitude = ang(1);
//            double heading = ang(2);
//            double c1 = cos(heading);
//            double s1 = sin(heading);
//            double c2 = cos(attitude);
//            double s2 = sin(attitude);
//            double c3 = cos(bank);
//            double s3 = sin(bank);
//            q.w() = sqrt(1.0 + c1 * c2 + c1 * c3 - s1 * s2 * s3 + c2 * c3) / 2.0;
//            double w4 = (4.0 * q.w());
//            q.x() = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
//            q.y() = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
//            q.z() = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
			return q;
		}

		/**
		 * update state(expressed as x)  based on residual (expressed as dx).
		 * @param x
		 * @param dx
		 * @return
		 */
		Eigen::Vector3d angleAdd(Eigen::Vector3d x, const Eigen::Vector3d &dx) {
//            Eigen::Quaterniond q, dq;
//            q = angle2q(x);
//            dq = angle2q(dx);
////            q = q * dq;
//            q.normalize();
//            x = q.toRotationMatrix().eulerAngles(0, 1, 2);
			Sophus::SO3d r = (Sophus::SO3d::exp(x));
			Sophus::SO3d tr = (Sophus::SO3d::exp(dx));
//            r = tr * r;

			r = r * tr;

			return r.log();
		}


		/**
		 * quaternion update function adopted in
		 * @tparam T
		 * @param q_in
		 * @param angle_velocity
		 * @param time_interval
		 * @return
		 */
		template<typename T>
		Eigen::Quaternion<T> quaternion_update(Eigen::Quaternion<T> q_in,
		                                       Eigen::Matrix<T, 3, 1> angle_velocity,
		                                       double time_interval) {
			Eigen::Matrix<T, 4, 1> tmp_q;
			tmp_q(3) = q_in.w();
			tmp_q(0) = q_in.x();
			tmp_q(1) = q_in.y();
			tmp_q(2) = q_in.z();

			Eigen::Matrix<T, 3, 1> tmp_w = angle_velocity * time_interval;
			T w_norm = tmp_w.norm();

			Eigen::Matrix<T, 4, 4> Theta;
			Theta.setZero();

			T c = 0.0;
			T s = 0.0;
//			c = 1 - w_norm * w_norm / 8.0 + pow(w_norm, 4.0) / 384.0;
//			s = 0.5 - w_norm * w_norm / 48.0;

			if (w_norm > 1e-6) {
				c = cos(w_norm / 2.0);
				s = 2 / w_norm * sin(w_norm / 2.0);

//				Theta << c, -tmp_w(0) * s, -tmp_w(1) * s, -tmp_w(2) * s,
//						tmp_w(0) * s, c, tmp_w(2) * s, -tmp_w(1),
//						tmp_w(1) * s, -tmp_w(2) * s, c, tmp_w(0) * s,
//						tmp_w(2) * s, tmp_w(1) * s, -tmp_w(0) * s, cy;
				double P = tmp_w(2);
				double Q = tmp_w(1);
				double R = tmp_w(0);
//				Theta << c, s * R, -s * Q, s * P,
//						-s * R, c, s * P, s * Q,
//						s * Q, -s * P, c, s * R,
//						-s * P, -s * Q, -s * R, c;
				Eigen::Matrix<T, 4, 4> OMEGA = Eigen::Matrix<T, 4, 4>::Identity();
				OMEGA << 0.0, R, -Q, P,
						-R, 0.0, P, Q,
						Q, -P, 0.0, R,
						-P, -Q, -R, 0.0;
				OMEGA = OMEGA * 0.5;

				tmp_q = (cos(w_norm / 2.0) * Eigen::Matrix<T, 4, 4>::Identity() +
				         2.0 / w_norm * sin(w_norm / 2.0) * OMEGA) * tmp_q;
				tmp_q = tmp_q / tmp_q.norm();
			}
			Eigen::Quaternion<double> q_out(tmp_q(0), tmp_q(1), tmp_q(2), tmp_q(3));

			return q_out;
		}


		/**
		 * eular angle
		 * @param ang
		 * @return rotation matrix.
		 */
		template<typename T>
		Eigen::Matrix<T, 3, 3> Rt2b(Eigen::Matrix<T, 3, 1> ang) {
			T cr(cos(ang[0])), sr(sin(ang[0]));
			T cp(cos(ang[1])), sp(sin(ang[1]));
			T cy(cos(ang[2])), sy(sin(ang[2]));

			Eigen::Matrix<T, 3, 3> R;

			R(0, 0) = cy * cp;
			R(0, 1) = sy * cp;
			R(0, 2) = -sp;

			R(1, 0) = -sy * cr + cy * sp * sr;
			R(1, 1) = cy * cr + sy * sp * sr;
			R(1, 2) = cp * sr;

			R(2, 0) = sy * sr + cy * sp * cr;
			R(2, 1) = -cy * sr + sy * sp * cr;
			R(2, 2) = cp * cr;

			return R;
		}


		/**
		 *  Rotation matrix to quanternions
		 * @param R rotation matrix
		 * @return quanternions
		 */
		template<typename Type>
		Eigen::Quaternion<Type> dcm2q(Eigen::Matrix<Type, 3, 3> R) {
			Type T(1.0 + R(0, 0) + R(1, 1) + R(2, 2));

			Type qw(0.0), qx(0.0), qy(0.0), qz(0.0);
			Type S(0.0);


			try {
				// 1e-3  ==>>>  fabs(T) != 0
				if (fabs(T) > 1e-8) {
					S = 0.5 / sqrt(fabs(T));

					qw = 0.25 / S;
					qx = (R(2, 1) - R(1, 2)) * S;
					qy = (R(0, 2) - R(2, 0)) * S;
					qz = (R(1, 0) - R(0, 1)) * S;

				} else {
					if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
						S = sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;

						qw = (R(2, 1) - R(1, 2)) / S;
						qx = 0.25 * S;
						qy = (R(0, 1) + R(1, 0)) / S;
						qz = (R(0, 2) + R(2, 0)) / S;
					} else if (R(1, 1) > R(2, 2)) {
						S = sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;

						qw = (R(0, 2) - R(2, 0)) / S;
						qx = (R(0, 1) + R(1, 0)) / S;
						qy = 0.25 * S;
						qz = (R(1, 2) + R(2, 1)) / S;
					} else {

						S = sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;

						qw = (R(1, 0) - R(0, 1)) / S;
						qx = (R(0, 2) + R(2, 0)) / S;
						qy = (R(1, 2) + R(2, 1)) / S;
						qz = 0.25 * S;

					}

				}


				Eigen::Quaternion<Type> q(qw, qx, qy, qz);
				q.normalize()/**/;

				return q;

			} catch (...) {
				std::cout << "THERE ARE SOME ERROR!" << std::endl;
				return Eigen::Quaternion<Type>(1.0, 0.0, 0.0, 0.0);
			}
		}


		/**
		 * Quternon to rotation matrix
		 * @param q quternion
		 * @return  rotation matrix
		 */
		template<typename T>
		Eigen::Matrix<T, 3, 3> q2dcm(Eigen::Quaternion<T> qua) {
			Eigen::Matrix<T, 4, 1> q;
			q(3) = qua.w();
			q(0) = qua.x();
			q(1) = qua.y();
			q(2) = qua.z();


			Eigen::Matrix<T, 6, 1> p;
			p.setZero();

			for (int i(0); i < 4; ++i) {
				p(i) = q(i) * q(i);
			}

			p(4) = p(1) + p(2);

			if (fabs(p(0) + p(3) + p(4)) > 1e-10) {
				p(5) = 2.0 / (p(0) + p(3) + p(4));

			} else {
				p(5) = 0.0;
			}


			Eigen::Matrix<T, 3, 3> R(Eigen::Matrix3d::Identity());
//        R.setZero();

			R(0, 0) = 1 - p(5) * p(4);
			R(1, 1) = 1 - p(5) * (p(0) + p(2));
			R(2, 2) = 1 - p(5) * (p(0) + p(1));

			p(0) = p(5) * q(0);
			p(1) = p(5) * q(1);
			p(4) = p(5) * q(2) * q(3);
			p(5) = p(0) * q(1);

			R(0, 1) = p(5) - p(4);
			R(1, 0) = p(5) + p(4);

			p(4) = p(1) * q(3);
			p(5) = p(0) * q(2);

			R(0, 2) = p(5) + p(4);
			R(2, 0) = p(5) - p(4);

			p(4) = p(0) * q(3);
			p(5) = p(1) * q(2);

			R(1, 2) = p(5) - p(4);
			R(2, 1) = p(5) + p(4);

			return R;
		}

		template<typename T>
		Eigen::Matrix<T, 3, 1> dcm2ang(Eigen::Matrix<T, 3, 3> r) {
			Eigen::Matrix<T, 3, 1> ang(0, 0, 0);
			ang(0) = atan2(r(2, 1), r(2, 2));
			ang(1) = atan(r(2, 0) / sqrt(1 - r(2, 0) * r(2, 0)));
			ang(2) = atan2(r(1, 0), r(0, 0));
			return ang;
		};


	};

};

#endif //COMPLEXITYPOSITIONING_IMUTOOLS_H
