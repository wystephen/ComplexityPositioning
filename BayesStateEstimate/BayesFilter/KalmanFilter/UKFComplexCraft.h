//
// Created by steve on 18-4-15.
//

#ifndef COMPLEXITYPOSITIONING_UKFCOMPLEXH_H
#define COMPLEXITYPOSITIONING_UKFCOMPLEXH_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <AWF.h>
#include <AuxiliaryTool/ImuTools.h>

namespace BSE {
	class UKFComplexCraft {
	public:

		UKFComplexCraft(Eigen::Matrix<double, 15, 15> init_prob) {
			prob_state_ = init_prob;
			std::cout << "initial prob:"
			          << init_prob << std::endl;
			state_x_.resize(15, 1);
			state_x_.setZero();

		}

		void update_function(Eigen::Matrix<double, 15, 1> &state,
		                     Eigen::Quaterniond &q,
		                     Eigen::Matrix<double, 6, 1> &input,
		                     double time_interval_,
		                     double local_g_) {
			q = ImuTools::quaternion_update<double>(q,
			                                        input.block(3, 0, 3, 1) + state.block(12, 0, 3, 1),
			                                        time_interval_);
//				std::cout << "time interval :" << time_interval_ << std::endl;

			Eigen::Vector3d acc = q * (input.block(0, 0, 3, 1) + state.block(9, 0, 3, 1)) +
			                      Eigen::Vector3d(0, 0, local_g_);

//				std::cout << "local g:" << local_g_ << " acc:" << acc.transpose() << std::endl;
			state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
			                          state.block(3, 0, 3, 1) * time_interval_;
			state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) + acc * time_interval_;

			state.block(6, 0, 3, 1) = ImuTools::dcm2ang<double>(q.toRotationMatrix());

//				state.block(9,0,6,1) =

		}

		/**
		 * @brief State update and state probability update based on Jacbian matrix.
		 * @param input
		 * @param noise_matrix
		 * @return
		 */
		Eigen::Matrix<double, 15, 1> StateTransIMU_jac(Eigen::Matrix<double, 6, 1> input,
		                                               Eigen::Matrix<double, 6, 6> noise_matrix) {

			Eigen::Quaterniond before_q = rotation_q_;

			rotation_q_ = ImuTools::quaternion_update<double>(rotation_q_,
			                                                  input.block(3, 0, 3, 1) + state_x_.block(12, 0, 3, 1),
			                                                  time_interval_);
//			rotation_q_ = ImuTools::quaternion_left_update<double>(rotation_q_,
//			                                                       input.block(3, 0, 3, 1) +
//			                                                       state_x_.block(12, 0, 3, 1),
//			                                                       time_interval_);

			rotation_q_.normalize();

			Eigen::Vector3d acc =
					ImuTools::q2dcm(rotation_q_) * (input.block(0, 0, 3, 1) + state_x_.block(9, 0, 3, 1)) +
					Eigen::Vector3d(0, 0, local_g_);
			state_x_.block(0, 0, 3, 1) = state_x_.block(0, 0, 3, 1) +
			                             state_x_.block(3, 0, 3, 1) * time_interval_;
			state_x_.block(3, 0, 3, 1) = state_x_.block(3, 0, 3, 1) + acc * time_interval_;

			state_x_.block(6, 0, 3, 1) = ImuTools::dcm2ang<double>(rotation_q_.toRotationMatrix());


			Eigen::Matrix3d Rb2t = ImuTools::q2dcm(rotation_q_);

			Eigen::Vector3d f_t = Rb2t * input.block(0, 0, 3, 1);

			Eigen::Matrix3d St;
			St << 0.0, -f_t(2), f_t(1),
					f_t(2), 0.0, -f_t(0),
					-f_t(1), f_t(0), 0.0;

			Eigen::Matrix3d O = Eigen::Matrix3d::Zero();

			Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

			Eigen::Matrix3d Da;//= Eigen::Matrix3d::diagonal(input.block(0, 0, 3, 1));

			Eigen::Matrix3d Dg;//= Eigen::Matrix3d::diagonal(input.block(3, 0, 3, 1));
			for (int i(0); i < 3; ++i) {
				Da(i, i) = input(i);
				Dg(i, i) = input(i + 3);
			}

			Eigen::Matrix3d B1 = O * 1.0;
			Eigen::Matrix3d B2 = O * 1.0;

			Eigen::Matrix<double, 15, 15> Fc;
			Fc.setZero();
			// x yz
			Fc.block(0, 3, 3, 3) = I;

			// vx vy vz
			Fc.block(3, 6, 3, 3) = St;
			Fc.block(3, 9, 3, 3) = Rb2t;

			// wx wy wz
			Fc.block(6, 12, 3, 3) = -1.0 * Rb2t;


			// bax bay baz
//			Fc.block(9,9,3,3) = I;


//			 bwx bwy bwz
//			Fc.block(12,12,3,3) = I;


			Eigen::Matrix<double, 15, 6> Gc;
			Gc.setZero();

			Gc.block(3, 0, 3, 3) = Rb2t;
			Gc.block(6, 3, 3, 3) = -1.0 * Rb2t;

			Eigen::Matrix<double, 15, 15> F = (Eigen::Matrix<double, 15, 15>::Identity());
//			F.setIdentity();
			F = F + Fc * time_interval_;


			Eigen::Matrix<double, 15, 6> G;
			G = time_interval_ * Gc;


			double old_p_norm = prob_state_.norm();
			prob_state_ = F * prob_state_ * F.transpose() +
			              G * noise_matrix * G.transpose();

			prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose());
			double new_p_norm = prob_state_.norm();


			if (std::isnan(prob_state_.sum())) {
				ERROR_MSG_FLAG("porb_state_ is nan.");
			}

			state_x_.block(6, 0, 3, 1) = ImuTools::dcm2ang(rotation_q_.toRotationMatrix());

			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
			logger_ptr_->addPlotEvent("ukf_craft_jac", "acc", input.block(0, 0, 3, 1));
			logger_ptr_->addPlotEvent("ukf_craft_jac", "acc_rotated", rotation_q_ * input.block(0, 0, 3, 1));
			logger_ptr_->addPlotEvent("ukf_craft_jac", "acc_linear", acc);
			logger_ptr_->addPlotEvent("ukf_craft_jac", "gyr", input.block(3, 0, 3, 1));

//			logger_ptr_->addPlotEvent("ukf_craft_jac_p", "p", prob_state_);

//			logger_ptr_->addPlotEvent("ukf_craft_jac", "before_q", before_q.toRotationMatrix().eulerAngles(0, 1, 2));
//			logger_ptr_->addPlotEvent("ukf_craft_jac", "input", input.block(3, 0, 3, 1));
//			logger_ptr_->addPlotEvent("ukf_craft_jac", "after_q", rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2));
//			logger_ptr_->addPlotEvent("ukf_craft_jac", "diff_q",
//			                          (before_q.inverse() * rotation_q_).toRotationMatrix().eulerAngles(0, 1, 2));
//
//			logger_ptr_->addPlotEvent("ukf_craft_jac", "pos", state_x_.block(0, 0, 3, 1));
//
//			logger_ptr_->addPlotEvent("craft_jac", "b_p_norm", old_p_norm);
//			logger_ptr_->addPlotEvent("craft_jac", "after_p_norm", new_p_norm);
//			logger_ptr_->addPlotEvent("craft_jac", "diff_p_norm", new_p_norm - old_p_norm);


			return state_x_;


		};


		/**
		 * state transaction function
		 * @param input
		 * @param noise_matrix
		 * @return
		 */
		Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
		                                           Eigen::Matrix<double, 6, 6> noise_matrix) {

//			std::cout << "time interval:" << time_interval_ << std::endl;
			Eigen::MatrixXd Sigma_matrix(prob_state_.rows() + noise_matrix.rows(),
			                             prob_state_.cols() + noise_matrix.cols());
			Sigma_matrix.setZero();
			Sigma_matrix.block(0, 0, prob_state_.rows(), prob_state_.cols()) = prob_state_;
			Sigma_matrix.block(prob_state_.rows(), prob_state_.cols(),
			                   noise_matrix.rows(), noise_matrix.cols()) = noise_matrix;
//			std::cout << "noise matrix \n"
//				<< noise_matrix << "\n";

			Eigen::MatrixXd L = (Sigma_matrix.llt().matrixL());


			int sigma_point_size = L.rows();

			std::vector<Eigen::VectorXd> state_stack(sigma_point_size * 2 + 2);
			std::vector<Eigen::Quaterniond> rotation_stack(sigma_point_size * 2 + 2);


			/**
			 * Update function for system state ( 15 -state model).
			 */
			auto update_function = [](Eigen::Matrix<double, 15, 1> &state,
			                          Eigen::Quaterniond &q,
			                          Eigen::Matrix<double, 6, 1> input,
			                          double time_interval_,
			                          double local_g_) {
				q = ImuTools::quaternion_update<double>(q,
				                                        input.block(3, 0, 3, 1) + state.block(12, 0, 3, 1),
				                                        time_interval_);
				q.normalize();
//				std::cout << "time interval :" << time_interval_ << std::endl;

				Eigen::Vector3d acc = q * (input.block(0, 0, 3, 1) + state.block(9, 0, 3, 1)) +
				                      Eigen::Vector3d(0, 0, local_g_);

//				std::cout << "local g:" << local_g_ << " acc:" << acc.transpose() << std::endl;
				state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
				                          state.block(3, 0, 3, 1) * time_interval_;
				state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) + acc * time_interval_;

				state.block(6, 0, 3, 1) = ImuTools::dcm2ang<double>(q.toRotationMatrix());

//				state.block(9,0,6,1) =

			};

//			state_stack[0] = upd

			Eigen::Matrix<double, 15, 1> tmp_state = state_x_;
			Eigen::Quaterniond tmp_q = rotation_q_;
			Eigen::Matrix<double, 6, 1> tmp_input = input * 1.0;

			update_function(tmp_state, tmp_q, tmp_input, time_interval_, local_g_);

			state_stack[0] = tmp_state * 1.0;
			state_stack[1] = tmp_state * 1.0;

			rotation_stack[0] = tmp_q;
			rotation_stack[1] = tmp_q;

			double coeff = std::sqrt(sigma_point_size + 1.0);


#pragma omp parallel for num_threads(12)
			for (int i = (0); i < sigma_point_size; ++i) {


				Eigen::Quaterniond tmp_q_plus = ImuTools::quaternion_update<double>(rotation_q_, L.block(6, i, 3, 1),
				                                                                    coeff);
				Eigen::Quaterniond tmp_q_minus = ImuTools::quaternion_update<double>(rotation_q_, L.block(6, i, 3, 1),
				                                                                     -1.0 * coeff);

				Eigen::Matrix<double, 21, 1> tmp_L = L.block(0, i, L.rows(), 1);


				Eigen::Matrix<double, 15, 1> tmp_L_state = tmp_L.block(0, 0, 15, 1);
				Eigen::Matrix<double, 6, 1> tmp_L_input = tmp_L.block(15, 0, 6, 1);


				Eigen::Matrix<double, 15, 1> tmp_state_plus = (tmp_state) + coeff * tmp_L_state;
				Eigen::Matrix<double, 15, 1> tmp_state_minus = (tmp_state) - coeff * tmp_L_state;


				Eigen::Matrix<double, 6, 1> tmp_input_plus = (tmp_input) + coeff * tmp_L_input;
				Eigen::Matrix<double, 6, 1> tmp_input_minus = (tmp_input) - coeff * tmp_L_input;





//				std::cout << "i:" << i << "before:" << tmp_state_plus.transpose() << "\n";
				update_function(tmp_state_plus, tmp_q_plus, tmp_input_plus, time_interval_, local_g_);
//				std::cout << "i:" << i << "after " << tmp_state_plus.transpose() << "\n";
				update_function(tmp_state_minus, tmp_q_minus, tmp_input_minus, time_interval_, local_g_);
//				std::cout << "k:" << i << "after" << tmp_state_minus.transpose() << "\n";

				state_stack[i + 2] = tmp_state_plus;
				state_stack[i + sigma_point_size + 2] = tmp_state_minus;

				rotation_stack[i + 2] = tmp_q_plus;
				rotation_stack[i + sigma_point_size + 2] = tmp_q_minus;
			}
//			std::cout << "----------------------------------------------------\n";


			double weight = 1.0 / (sigma_point_size * 2.0 + 2.0);
			// compute average rotation.
			Eigen::Quaterniond average_q(0, 0, 0, 0);


			/**
			 * Hard way for quaternion average
			 */
			Eigen::Matrix<double, 4, Eigen::Dynamic> Q(4, rotation_stack.size());
			for (int i = 0; i < rotation_stack.size(); ++i) {
				Q.col(i) = weight * rotation_stack[i].coeffs();
			}

			Eigen::Matrix<double, 4, 4> QQt = Q * Q.transpose();

			Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(QQt);

			std::complex<double> max_eigenvalue = es.eigenvalues()[0];
			Eigen::Matrix<std::complex<double>, 4, 1> max_eigenvector =
					es.eigenvectors().col(0);
			for (int i = 1; i < 4; i++) {
				if (std::norm(es.eigenvalues()[i]) > std::norm(max_eigenvalue)) {
					max_eigenvalue = es.eigenvalues()[i];
					max_eigenvector = es.eigenvectors().col(i);
				}
			}
			average_q.coeffs()
					<<                //
					max_eigenvector[0].real(),  //
					max_eigenvector[1].real(),  //
					max_eigenvector[2].real(),  //
					max_eigenvector[3].real();


			rotation_q_ = average_q;

			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
			logger_ptr_->addPlotEvent("ukf_craft", "quaternion", rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2));


			// compute average state
			state_x_.setZero();
//			double weight_sum  = 0.0;
			for (const auto state: state_stack) {
				state_x_ += weight * state;
//				weight_sum+=weight;

			}
			state_x_.block(6, 0, 3, 1) = ImuTools::dcm2ang<double>(average_q.toRotationMatrix());
//			logger_ptr_->addPlotEvent("uwk_craft_weight","weight_sum",weight_sum);



			// compute probability.

			double before_p_norm = prob_state_.norm();
			prob_state_.setZero();
			for (int i(0); i < state_stack.size(); ++i) {
				Eigen::Matrix<double, 15, 1> state = state_stack[i];
				Eigen::Quaterniond the_q = rotation_stack[i];

				Eigen::Matrix<double, 15, 1> dx = state - state_x_;
				Eigen::Quaterniond d_q = average_q.inverse() * the_q;
				Eigen::Matrix<double, 3, 1> t3d = ImuTools::dcm2ang(d_q.toRotationMatrix());

//				dx.block(6, 0, 3, 1) = t3d;

				prob_state_ += weight * dx * dx.transpose();

				logger_ptr_->addPlotEvent("ukf_craft_diff_x","d"+std::to_string(i),dx);

			}

			prob_state_ = 0.5 * (prob_state_ * prob_state_.transpose());
//			prob_state_ = 0.5 * (prob_state_.eval() + prob_state_.transpose().eval());



			auto logger_ptr = AWF::AlgorithmLogger::getInstance();
			logger_ptr->addPlotEvent("ukf", "probability", prob_state_);


			double after_p_norm = prob_state_.norm();
			if (after_p_norm > 10.0 * before_p_norm && after_p_norm > 4.0) {


				std::cout << "average rotation:" << average_q.w()
				          << "," << average_q.x()
				          << "," << average_q.y()
				          << "," << average_q.z() << std::endl;
				for (auto q :rotation_stack) {
					std::cout << q.w()
					          << ","
					          << q.x()
					          << ","
					          << q.y()
					          << ","
					          << q.z() << std::endl;
				}
				std::cout << "p norm is really big:"
				          << after_p_norm << std::endl;
			}


//			logger_ptr->addPlotEvent("ukf_state_craft", "state", state_x_);
//			logger_ptr->addPlotEvent("ukf_craft", "P", prob_state_);
			logger_ptr->addPlotEvent("ukf_craft", "rotated acc", rotation_q_ * input.block(0, 0, 3, 1));
			logger_ptr->addPlotEvent("ukf_craft", "q_norm", rotation_q_.norm());

			return state_x_;

		};


		/**
		   * zero velocity measuremnt upd
		   * @param cov_matrix
		   */
		void MeasurementStateZV(Eigen::Matrix3d cov_matrix) {
			H_ = Eigen::MatrixXd::Zero(3, state_x_.rows());
			H_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * 1.0;
			bool IS_DEBUG = false;
			if (IS_DEBUG) {
				std::cout << H_ << std::endl;
				std::cout << " p * H^T :" << prob_state_ * H_.transpose().eval() << std::endl;
				std::cout << " H * P * H^T + cosv:" << H_ * prob_state_ * H_.transpose().eval() + cov_matrix
				          << std::endl;
				std::cout << "inverse " << (H_ * prob_state_ * H_.transpose().eval() + cov_matrix).inverse()
				          << std::endl;

			}

			K_ = (prob_state_ * H_.transpose().eval()) *
			     (H_ * prob_state_ * H_.transpose().eval() + cov_matrix).inverse();

			if (std::isnan(K_.sum()) || std::isinf(K_.sum())) {
				std::cout << "K is nan" << std::endl;
			}

			/*
			 * update probability
			 */
			double before_p_norm = prob_state_.norm();
			prob_state_ = (Eigen::Matrix<double, 15, 15>::Identity() - K_ * H_) * prob_state_;
			prob_state_ = (prob_state_ + prob_state_.transpose().eval()) * 0.5;
			double after_p_norm = prob_state_.norm();

			if (prob_state_.norm() > 10000) {
				ERROR_MSG_FLAG("probability is too big,its norm is:" + std::to_string(prob_state_.norm()));
				prob_state_ /= 100.0;
			}
			if (std::isnan(prob_state_.sum()) || std::isinf(prob_state_.sum())) {
				std::cout << "state prob has nan" << std::endl;
			}


			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
			logger_ptr_->addPlotEvent("ukf_craft", "angle_before", rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2));
			Eigen::Quaterniond tmp_before_q = rotation_q_;
			/*
			 * update state
			 */
			Eigen::Vector3d m(0, 0, 0); // pseudo velocity measurement.
			dX_ = K_ * (m - state_x_.block(3, 0, 3, 1));

			state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) + dX_.block(0, 0, 6, 1);

			Eigen::Vector3d epsilon(dX_(6), dX_(7), dX_(8));


			rotation_q_ = ImuTools::quaternion_left_update(rotation_q_, epsilon, -1.0);

			state_x_.block(6, 0, 3, 1) = ImuTools::dcm2ang<double>(ImuTools::q2dcm(rotation_q_));

			state_x_.block(9, 0, 6, 1) = state_x_.block(9, 0, 6, 1) + dX_.block(9, 0, 6, 1);

			logger_ptr_->addPlotEvent("complexfull", "offset_acc", state_x_.block(9, 0, 3, 1));
			logger_ptr_->addPlotEvent("complexfull", "offset_gyr", state_x_.block(12, 0, 3, 1));

			logger_ptr_->addPlotEvent("ukf_craft_zv", "P", prob_state_);


			logger_ptr_->addPlotEvent("ukf_craft_dx","dx",dX_);
//			logger_ptr_->addPlotEvent("ukf_craft", "epsilon", epsilon);
//			logger_ptr_->addPlotEvent("ukf_craft", "angle_before",
//			                          tmp_before_q.toRotationMatrix().eulerAngles(0, 1, 2));
//			logger_ptr_->addPlotEvent("ukf_craft", "angle_after", rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2));
//			logger_ptr_->addPlotEvent("ukf_craft", "angle_diff", (rotation_q_.inverse() *
//			                                                      tmp_before_q).toRotationMatrix().eulerAngles(0, 1,
//			                                                                                                   2));

//			logger_ptr_->addPlotEvent("ukf_craft_state", "pos", state_x_.block(0, 0, 3, 1));
//			logger_ptr_->addPlotEvent("ukf_craft_state", "vel", state_x_.block(3, 0, 3, 1));


//			logger_ptr_->addPlotEvent("craft_zv", "b_p_norm", before_p_norm);
//			logger_ptr_->addPlotEvent("craft_zv", "after_p_norm", after_p_norm);
//			logger_ptr_->addPlotEvent("craft_zv", "diff_p_norm", after_p_norm - before_p_norm);

		}


		/**
		 * @brief
		 * @param pose
		 * @param cov_m
		 */
		void MeasurementUwbPose(Eigen::Matrix<double, 3, 1> pose,
		                        Eigen::Matrix<double, 3, 3> cov_m) {
			H_.resize(3, state_x_.rows());
			H_.setZero();
			H_.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

			K_ = (prob_state_ * H_.transpose()) *
			     (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

			K_ = (prob_state_ * H_.transpose()) *
			     (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

			dX_ = K_ * (pose - H_ * state_x_);

			state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) + dX_.block(0, 0, 6, 1);


			Eigen::Vector3d epsilon(dX_(6), dX_(7), dX_(8));
			rotation_q_ = ImuTools::quaternion_left_update(rotation_q_, epsilon, -1.0);
//			state_x_.block(6, 0, 3, 1) = r.log();

			state_x_.block(9, 0, 6, 1) = state_x_.block(9, 0, 6, 1) + dX_.block(9, 0, 6, 1);

			prob_state_ = (Eigen::Matrix<double, 15, 15>::Identity() - K_ * H_) * prob_state_;
			prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose());

			return;
		}


		/**
 * @brief
 * @param pose
 * @param cov_m
 */
		void MeasurementUwb(Eigen::Matrix<double, 4, 1> input,
		                    Eigen::Matrix<double, 1, 1> cov_m) {
			Eigen::Vector3d b = input.block(0, 0, 3, 1);
			Eigen::Matrix<double, 1, 1> z;
			z(0) = input(3);
			Eigen::Matrix<double, 1, 1> y;
			y(0) = (state_x_.block(0, 0, 3, 1) - b).norm();

			H_.resize(1, state_x_.rows());
			H_.setZero();
			H_.block(0, 0, 1, 3) = (state_x_.block(0, 0, 3, 1) - b).transpose() / y(0);

			K_ = (prob_state_ * H_.transpose()) *
			     (H_ * prob_state_ * H_.transpose() + cov_m).inverse();

			dX_ = K_ * (z - y);

			state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) + dX_.block(0, 0, 6, 1);


			Eigen::Vector3d epsilon(dX_(6), dX_(7), dX_(8));
			rotation_q_ = ImuTools::quaternion_left_update(rotation_q_, epsilon, -1.0);
//			state_x_.block(6, 0, 3, 1) = r.log();

			state_x_.block(9, 0, 6, 1) = state_x_.block(9, 0, 6, 1) + dX_.block(9, 0, 6, 1);

			prob_state_ = (Eigen::Matrix<double, 15, 15>::Identity() - K_ * H_) * prob_state_;
			prob_state_ = 0.5 * (prob_state_ + prob_state_.transpose());

			return;
		}


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
			std::cout << "eular angle:" << rotation_q_.toRotationMatrix().eulerAngles(0, 1, 2).transpose()
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
