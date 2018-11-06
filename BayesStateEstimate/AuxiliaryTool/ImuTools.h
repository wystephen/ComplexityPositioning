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


//#include "AWF.h"

namespace BSE {
	namespace ImuTools {

		/**
		 * zero velocity detector based on GLRT algorithm.
		 * @param u
		 * @return
		 */
		bool GLRT_Detector(Eigen::MatrixXd u,
		                   double sigma = 0.1,
		                   double sigma_gyr = 0.1 * M_PI / 100.0) {
			if (u.cols() == 6 && u.rows() != 6) {
				Eigen::MatrixXd tu = u * 1.0;
//        u = u.transpose();
				u = tu.transpose();
			}
			assert(u.rows() == 6 || "u must be a 6 rows matrix(each col represent acc and gyro at one moement");
			Eigen::Vector3d ya_m;
			double g = 9.8;
			double sigma_a_ = sigma;
//			double sigma_g_ = sigma_gyr * M_PI / 180.0;
			double sigma_g_ = sigma_gyr;
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
//			imu_data.resize(row, 1 + 3 + 3 + 3 + 1);//
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


			q = q.inverse();

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
		 * Hat of 3-dimension vector
		 * @tparam T type of data internal.
		 * @param v 3-dimension vector
		 * @return
		 */
		template<typename T>
		Eigen::Matrix<T, 3, 3> hat(Eigen::Matrix<T, 3, 1> v) {
			Eigen::Matrix<T, 3, 3> v_hat;
			v_hat << 0.0, -v(2), v(1),
					v(2), 0.0, -v(0),
					-v(1), v(0), 0.0;
			return v_hat;
		};

		/**
		 * @brief
		 * @tparam T
		 * @param q_in
		 * @param update_angle
		 * @param coeff
		 * @return
		 */
		template<typename T>
		Eigen::Quaternion<T> quaternion_left_update(Eigen::Quaternion<T> q_in,
		                                            Eigen::Matrix<T, 3, 1> update_angle,
		                                            double coeff) {
			Eigen::Matrix<T, 4, 1> tmp_q;
			tmp_q(0) = q_in.w();
			tmp_q(1) = q_in.x();
			tmp_q(2) = q_in.y();
			tmp_q(3) = q_in.z();

			Eigen::Matrix<T, 3, 1> eta = update_angle * coeff * 0.5;
			T eta_norm = eta.norm();

			Eigen::Matrix<T, 4, 1> mul_q;
			Eigen::Matrix<T, 4, 4> q_L;
//			q_L.setIdentity();
			if (eta_norm > 1e-18) {

				mul_q(0) = cos(eta_norm);
				mul_q.block(1, 0, 3, 1) = eta * sin(eta_norm) / eta_norm;


			} else {
				mul_q(0) = 1.0;
				mul_q.block(1, 0, 3, 1) = eta;
			}

			q_L << mul_q(0), -mul_q(1), -mul_q(2), -mul_q(3),
					mul_q(1), mul_q(0), -mul_q(3), mul_q(2),
					mul_q(2), mul_q(3), mul_q(0), -mul_q(1),
					mul_q(3), -mul_q(2), mul_q(1), mul_q(0);

			tmp_q = q_L * tmp_q;
			q_in.w() = tmp_q(0);
			q_in.x() = tmp_q(1);
			q_in.y() = tmp_q(2);
			q_in.z() = tmp_q(3);

			q_in.normalize();

			return q_in;
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
			/**
			 * Typical update way of quaternion.
			 * */
			Eigen::Matrix<T, 4, 1> tmp_q;
			tmp_q(0) = q_in.w();
			tmp_q(1) = q_in.x();
			tmp_q(2) = q_in.y();
			tmp_q(3) = q_in.z();


			Eigen::Matrix<T, 4, 1> mul_q(1.0, 0.0, 0.0, 0.0);
			Eigen::Matrix<T, 3, 1> angle_delta = angle_velocity * time_interval;

			if (angle_delta.norm() > 1e-18) {
				mul_q.block(1, 0, 3, 1) = angle_delta * 0.5;

//				mul_q(0) = cos(angle_delta.norm() / 2.0);
//				mul_q.block(1, 0, 3, 1) = angle_delta * 1.0 / angle_delta.norm() * sin(angle_delta.norm() / 2.0);

				Eigen::Matrix<T, 4, 4> q_R;
				q_R(0, 0) = mul_q(0);
//				q_R = q_R + Eigen::Matrix<T, 4, 4>::Identity() * mul_q(0);
				q_R.block(1, 0, 3, 1) = mul_q.block(1, 0, 3, 1);
				q_R.block(0, 1, 1, 3) = -1.0 * mul_q.block(1, 0, 3, 1).transpose();
				q_R.block(1, 1, 3, 3) =
						Eigen::Matrix<T, 3, 3>::Identity() * mul_q(0) - hat<double>(mul_q.block(1, 0, 3, 1));

				tmp_q = q_R * tmp_q;

			}

//			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
//			logger_ptr_->addPlotEvent("quaternion_update", "q_norm", tmp_q.norm());
//			logger_ptr_->addPlotEvent("quaternion_update", "q_before_norm", q_in.norm());


			Eigen::Quaternion<T> q_out(tmp_q(0), tmp_q(1), tmp_q(2), tmp_q(3));
			q_out.normalize();
			return q_out;


			/**
			 * @brief Note a good way (tested by cumulate samll draft)
			 */
//			Eigen::Matrix<T, 3, 1> eta = angle_velocity * 0.5;
//			T eta_norm = eta.norm();
//
//			Eigen::Quaternion<T> mul_q;
//			if (eta_norm < 1e8) {
//
//				mul_q.w() = 1.0;
//				mul_q.x() = eta(0);
//				mul_q.y() = eta(1);
//				mul_q.z() = eta(2);
//
//			} else {
//
//				mul_q.w() = cos(eta_norm);
//				mul_q.x() = eta(0) * sin(eta_norm) / eta_norm;
//				mul_q.y() = eta(1) * sin(eta_norm) / eta_norm;
//				mul_q.z() = eta(2) * sin(eta_norm) / eta_norm;
//
//
//			}
//			Eigen::Quaternion<T> out_q = mul_q * q_in;
//			out_q.normalize();
//			return out_q;

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
//			qua.normalize();
//			return qua.toRotationMatrix();
		}

		template<typename T>
		Eigen::Matrix<T, 3, 1> dcm2ang(Eigen::Matrix<T, 3, 3> r) {
			Eigen::Matrix<T, 3, 1> ang(0, 0, 0);
			ang(0) = atan2(r(2, 1), r(2, 2));
			ang(1) = atan2(r(2, 0), sqrt(1 - r(2, 0) * r(2, 0)));
			ang(2) = atan2(r(1, 0), r(0, 0));
			if (std::isnan(ang.norm())) {
				ERROR_MSG_FLAG("angle with nana in dcm2ang function");
				std::cout << "ang:"
				          << ang.transpose()
				          << "\nrotation matrix;"
				          << r
				          << "\n"
				          << "r*r^T"
				          << r * r.transpose()
				          << "\n";
			}
			return ang;
		}

		/**
		 * @brief build up transpose matrix throught quaternion and pos.
		 * @tparam T
		 * @param pos
		 * @param rotation_q
		 * @return
		 */
		template<typename T>
		Eigen::Isometry3d build_transform_matrix(Eigen::Matrix<T, 3, 1> pos,
		                                         Eigen::Quaternion<T> rotation_q) {
			Eigen::Isometry3d t_mat = Eigen::Isometry3d::Identity();
			t_mat.matrix().block(0, 0, 3, 3) = q2dcm(rotation_q);
			t_mat.matrix().block(0, 3, 3, 1) = pos;

			return t_mat;
		};


		/**
		 * @brief return the right quaternion based on acc measurement at initial state.
		 * @param imu_data
		 * @param initial_ori
		 * @param debug_flag
		 * @return
		 */
		Eigen::Quaterniond initial_quaternion(Eigen::MatrixXd imu_data,
		                                      double initial_ori,
		                                      bool debug_flag = false) {
			Eigen::Vector3d acc = imu_data.block(0, 0, imu_data.rows(), 3).colwise().mean().transpose();
			auto g = acc.norm();

			auto local_g_ = -1.0 * g;


			auto g_error = [&, acc](double roll, double pitch, double yaw) -> double {

				auto rotate_matrix = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
				                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
				                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
				return std::abs(std::abs(g) * 1.0 * local_g_ / std::abs(local_g_) + (rotate_matrix * acc)(2));
			};
			auto ge(0.0);

			/**
			 * find initial euler angle through optimization.
			 */
			double tr = 0.0;
			double tp = 0.0;

			double step_len = 0.000005;
			double update_rate = 0.5;
			int iter_counter = 0;
			double current_error(g_error(tr, tp, initial_ori));

			if (debug_flag) {

				std::cout << "start gravity error function:" << tr
				          << "," << tp << "," << initial_ori << "," << current_error << std::endl;

			}

			while (current_error > 1e-7 && iter_counter < 30000) {

				iter_counter++;
//                tr +=(g_error)
				// compute gradient
				double delta_tr = (g_error(tr + step_len, tp, initial_ori) - current_error) / step_len;
				double delta_tp = (g_error(tr, tp + step_len, initial_ori) - current_error) / step_len;
				if (std::isnan(delta_tp) || std::isnan(delta_tr)) {
					delta_tp = 0.0001;
					delta_tr = 0.0001;

					continue;
				}
				// update state.
				tr -= delta_tr * update_rate;
				tp -= delta_tp * update_rate;

				while (tr > M_PI + 0.01) {
					tr -= 2.0 * M_PI;
				}
				while (tr < -M_PI - 0.01) {

					tr += 2.0 * M_PI;
				}

				while (tp > M_PI + 0.01) {
					tp -= 2.0 * M_PI;
				}

				while (tp < -M_PI - 0.01) {
					tp += 2.0 * M_PI;
				}
				/*
				 * Reduce the learning rate.
				 */
				if (update_rate > 0.00001) {
					update_rate *= 0.99;
				}


				current_error = g_error(tr, tp, initial_ori);

			}


			if (debug_flag) {

				std::cout << "end gravity error function:" << tr
				          << "," << tp << "," << initial_ori << "," << current_error << std::endl;

			}

			Eigen::Quaterniond rotation_q_ = (Eigen::AngleAxisd(tr, Eigen::Vector3d::UnitX())
			                                  * Eigen::AngleAxisd(tp, Eigen::Vector3d::UnitY())
			                                  * Eigen::AngleAxisd(initial_ori, Eigen::Vector3d::UnitZ()));

			return rotation_q_;

		}


	};

};

#endif //COMPLEXITYPOSITIONING_IMUTOOLS_H
