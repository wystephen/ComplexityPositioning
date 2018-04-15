//
// Created by steve on 18-4-15.
//

#ifndef COMPLEXITYPOSITIONING_UKFCOMPLEXH_H
#define COMPLEXITYPOSITIONING_UKFCOMPLEXH_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <AuxiliaryTool/ImuTools.h>

namespace BSE {
	class UKFComplexCraft {
	public:

		UKFComplexCraft(Eigen::Matrix<double, 15, 15> init_prob) {
			prob_state_ = init_prob;
			state_x_.resize(15, 1);
			state_x_.setZero();

		}


		/**
		 * state transaction function
		 * @param input
		 * @param noise_matrix
		 * @return
		 */
		Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
		                                           Eigen::Matrix<double, 6, 6> noise_matrix) {

			Eigen::MatrixXd Sigma_matrix(prob_state_.rows() + noise_matrix.rows(),
			                             prob_state_.cols() + noise_matrix.cols());
			Sigma_matrix.setZero();
			Sigma_matrix.block(0, 0, prob_state_.rows(), prob_state_.cols()) = prob_state_;
			Sigma_matrix.block(prob_state_.rows(), prob_state_.cols(),
			                   noise_matrix.rows(), noise_matrix.cols()) = noise_matrix;

			Eigen::MatrixXd L = (Sigma_matrix.llt().matrixL());

			int sigma_point_size = L.rows();

			std::vector<Eigen::VectorXd> state_stack(sigma_point_size * 2 + 2);
			std::vector<Eigen::Quaterniond> rotation_stack(sigma_point_size * 2 + 2);


		};


		/**
		 * initial own pose.
		 * @param imu_data
		 * @param initial_ori
		 * @param initial_pos
		 * @return
		 */
		bool initial_state(Eigen::MatrixXd imu_data,
		                   double initial_ori = 0.0,
		                   Eigen::Vector3d initial_pos = Eigen::Vector3d(0, 0, 0)
		) {
			long double f_u(0.0), f_v(0.0), f_w(0.0);
			Eigen::Vector3d acc = imu_data.block(0, 0, imu_data.rows(), 3).colwise().mean();
			auto g = acc.norm();
			local_g_ = -1.0 * g;


			auto g_error = [&, &g, acc](double roll, double pitch, double yaw) -> double {

				auto rotate_matrix = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
				                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
				                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
				return std::abs(std::abs(g) * 1.0 * local_g_ / std::abs(local_g_) + (rotate_matrix * acc)(2));
			};
			auto ge(0.0);
//

			/**
			 * find initial euler angle through optimization.
			 */
			double tr = 0.0;
			double tp = 0.0;

			double step_len = 0.000005;
			double update_rate = 0.5;
			int iter_counter = 0;
			double current_error(g_error(tr, tp, initial_ori));
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


			state_x_.block(0, 0, 3, 1) = initial_pos;
			state_x_.block(3, 0, 3, 1).setZero();
			state_x_.block(6, 0, 3, 1) = Eigen::Vector3d(tr, tp, initial_ori);

			rotation_q_ = (Eigen::AngleAxisd(tr, Eigen::Vector3d::UnitX())
			               * Eigen::AngleAxisd(tp, Eigen::Vector3d::UnitY())
			               * Eigen::AngleAxisd(initial_ori, Eigen::Vector3d::UnitZ()));

			std::cout << "value angle:" << state_x_.block(6, 0, 3, 1).transpose() << std::endl;
			std::cout << "eular angle:" << rotate_q_.toRotationMatrix().eulerAngles(0, 1, 2).transpose()
			          << std::endl;
			std::cout << "before acc:" << acc.transpose() << std::endl;
			std::cout << "after acc:" << (rotation_q_ * acc).transpose() << std::endl;


		}

		/////////////////////////
		Eigen::MatrixXd state_x_;
		Eigen::MatrixXd prob_state_;
		Eigen::MatrixXd input_;
		Eigen::MatrixXd m_;


		/////////////////////

		/**
		 * X_i=A*X_{i-1}+B*u_i+w_i
		 * z_i=H*X_i+v_i
		 * w_i \in Q
		 * v_i \in R
		 */
		Eigen::MatrixXd A_ = Eigen::MatrixXd();
		Eigen::MatrixXd B_ = Eigen::MatrixXd();
		Eigen::MatrixXd H_ = Eigen::MatrixXd();
		Eigen::MatrixXd Q_ = Eigen::MatrixXd();
		Eigen::MatrixXd R_ = Eigen::MatrixXd();
		Eigen::MatrixXd K_ = Eigen::MatrixXd();
		Eigen::MatrixXd dX_ = Eigen::MatrixXd();


		///////////////////////
		double time_interval_ = 0.005;

		Eigen::Quaterniond rotation_q_ = Eigen::Quaterniond().setIdentity();

		double local_g_ = -9.81;

		std::string class_name_ = "UKFComplexCraft";

	};
}


#endif //COMPLEXITYPOSITIONING_UKFCOMPLEXH_H
