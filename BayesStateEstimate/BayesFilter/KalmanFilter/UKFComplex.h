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
// Created by steve on 18-4-7.
//

#ifndef COMPLEXITYPOSITIONING_UKFCOMPLEX_H
#define COMPLEXITYPOSITIONING_UKFCOMPLEX_H


#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <sophus/average.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <AuxiliaryTool/GravityOrientationFunction.h>
#include <AuxiliaryTool/ImuUpdateFunction.h>
#include <AuxiliaryTool/SimpleImuUpdateFunction.h>
#include <AuxiliaryTool/MagMeasurementFunction.h>
#include <AuxiliaryTool/MagGravityMeasurementFunction.h>
#include <AuxiliaryTool/ImuTools.h>
#include <AuxiliaryTool/UwbMeasurementFunction.h>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.h"
#include "KFComplex.h"

#include "omp.h"

namespace BSE {
	class UKFComplex : public KFComplex {
	public:
		/**
		 * 15-state UKF for IMU
		 */
		UKFComplex(Eigen::Matrix<double, 15, 15> init_prob) :
				KFComplex(init_prob.block(0, 0, 9, 9)) {
			prob_state_ = init_prob;
			state_x_.setZero();
			class_name_ = "UKFComplexFull";

		};

		void initial_state(Eigen::MatrixXd imu_data,
		                   double initial_ori = 0.0,
		                   Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {
			KFComplex::initial_state(imu_data, initial_ori, initial_pose);
			state_x_.block(9, 0, 6, 1).setZero();
			std::cout << "UKFComplex initialized " << std::endl;

		}

		/**
		 * state transaction function.
		 * @param input
		 * @param noise_matrix
		 * @return
		 */
		Eigen::Matrix<double, 15, 1> StateTransIMU(Eigen::Matrix<double, 6, 1> input,
		                                           Eigen::Matrix<double, 6, 6> noise_matrix) {

			// construct  lower-triangular matrix of P(state_prob) and Q(noise_matrix)

			Eigen::MatrixXd Sigma_matrix(prob_state_.rows() + noise_matrix.rows(),
			                             prob_state_.cols() + noise_matrix.cols());
			Sigma_matrix.setZero();
			Sigma_matrix.block(0, 0, prob_state_.rows(), prob_state_.cols()) = prob_state_;
			Sigma_matrix.block(prob_state_.rows(), prob_state_.cols(), noise_matrix.rows(),
			                   noise_matrix.cols()) = noise_matrix;

			// lower-triangle matrix
//			auto L = Sigma_matrix.triangularView<Eigen::Lower>();

			Eigen::MatrixXd L(Sigma_matrix.llt().matrixL());

			int sigma_point_size = L.rows();// number of sigma point.

			std::vector<Eigen::VectorXd> state_stack(sigma_point_size * 2 + 2);
			std::vector<Sophus::SO3d> rotation_stack(sigma_point_size * 2 + 2);

			auto siuf = FullImuUpdateFunction(rbn_, time_interval_, local_g_);

			state_stack[0] = siuf.compute(state_x_, input);
			state_stack[1] = state_stack[1];

			rotation_stack[0] = Sophus::SO3d::exp(state_stack[0].block(6, 0, 3, 1));
			rotation_stack[1] = Sophus::SO3d::exp(state_stack[1].block(6, 0, 3, 1));


			double coeff = std::sqrt(sigma_point_size + 1);

			for (int i(0); i < sigma_point_size; ++i) {
				state_stack[i + 2] = siuf.compute(state_x_ + L.block(0, i, state_x_.rows(), 1) * coeff,
				                                  input + L.block(state_x_.rows(), i, input.rows(), 1) * coeff);
				state_stack[i + sigma_point_size + 2] = siuf.compute(
						state_x_ - L.block(0, i, state_x_.rows(), 1) * coeff,
						input - L.block(state_x_.rows(), i, input.rows(), 1) * coeff);

				rotation_stack[i + 2] = Sophus::SO3d::exp(state_stack[i + 2].block(6, 0, 3, 1));
				rotation_stack[i + sigma_point_size + 2] = Sophus::SO3d::exp(
						state_stack[i + 2 + sigma_point_size].block(6, 0, 3, 1));

			}


			// compute state
			auto average_roation = Sophus::average<std::vector<Sophus::SO3d>>(rotation_stack);
//			if(average_roation.bool())

			state_x_.setZero();
			for (auto state :state_stack) {
				state_x_ += double(1. / (sigma_point_size * 2.0 + 2.0)) * state;
			}


			state_x_.block(6, 0, 3, 1) = average_roation->log();

			prob_state_.setZero();
			for (auto state :state_stack) {
				auto dx = state - state_x_;
				prob_state_ += double(1. / (sigma_point_size * 2.0 + 2.0)) * dx * dx.transpose();
			}

			return state_x_;


		};

		/**
		   * zero velocity measuremnt upd
		   * @param cov_matrix
		   */
		void MeasurementStateZV(Eigen::Matrix3d cov_matrix) {
			H_ = Eigen::MatrixXd::Zero(3, state_x_.rows());
			H_.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * 1.0;
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
			auto identity_matrix = Eigen::MatrixXd(state_x_.rows(), state_x_.rows());
			identity_matrix.setZero();
			prob_state_ = (Eigen::Matrix<double, 15, 15>::Identity() - K_ * H_) * prob_state_;
			prob_state_ = (prob_state_ + prob_state_.transpose().eval()) * 0.5;
			if (prob_state_.norm() > 10000) {
				std::cout << __FILE__
				          << ":"
				          << __LINE__
				          << " Error state prob is too big"
				          << prob_state_.norm()
				          << std::endl;
				prob_state_ /= 100.0;
			}
			if (std::isnan(prob_state_.sum()) || std::isinf(prob_state_.sum())) {
				std::cout << "state prob has nan" << std::endl;
			}

			/*
			 * update state
			 */
			Eigen::Vector3d m(0, 0, 0); // pseudo velocity measurement.
			dX_ = K_ * (m - state_x_.block(3, 0, 3, 1));

			state_x_.block(0, 0, 6, 1) = state_x_.block(0, 0, 6, 1) + dX_.block(0, 0, 6, 1);

			rbn_ = Sophus::SO3d::exp(state_x_.block(6, 0, 3, 1));
//            rbn_ = Sophus::SO3d::exp(dX_.block(6, 0, 3, 1)) * rbn_;
			rbn_ = rbn_ * Sophus::SO3d::exp(dX_.block(6, 0, 3, 1));
			state_x_.block(6, 0, 3, 1) = rbn_.log();
			state_x_.block(9, 0, 6, 1) = state_x_.block(9, 0, 6, 1) + dX_.block(9, 0, 6, 1);

			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
			logger_ptr_->addPlotEvent("complexfull", "offset_acc", state_x_.block(9, 0, 3, 1));
			logger_ptr_->addPlotEvent("complexfull", "offset_gyr", state_x_.block(12, 0, 3, 1));


		}

	};


}

#endif //COMPLEXITYPOSITIONING_UKFCOMPLEX_H
