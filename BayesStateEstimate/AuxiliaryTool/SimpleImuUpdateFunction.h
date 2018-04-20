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
// Created by steve on 18-3-9.
//

#ifndef COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
#define COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H

#include <AWF.h>
#include "ImuUpdateFunction.h"

#include <BSE.h>
#include "ImuTools.h"

#include <sophus/so3.hpp>


namespace BSE {
	class FFImuUpdateFunction :
			public ImuUpdateFunction {
	public:
		/**
		 *
		 * @param so3
		 * @param time_interval
		 * @param local_gravity
		 */
		FFImuUpdateFunction(Sophus::SO3d &so3,
		                    double time_interval,
		                    double local_gravity
		) :
				ImuUpdateFunction(21,
				                  time_interval,
				                  local_gravity) {
			epsilon_ = 1e-8;
			rbn_ = so3;

		}


		/**
		 * Core
		 * @param state
		 * @param input
		 * @return
		 */
		Eigen::MatrixXd compute(Eigen::MatrixXd state,
		                        Eigen::MatrixXd input) {
			Eigen::MatrixXd out_state(state.rows(), 1);

			auto rotation = Sophus::SO3d::exp(state.block(6, 0, 3, 1));

			/// vector to matrix (vector as diag).
			auto v2m = [](Eigen::VectorXd &&v) -> Eigen::MatrixXd {
				Eigen::MatrixXd m(v.rows(), v.rows());
				m.setZero();
				for (int i(0); i < v.rows(); ++i) {
					m(i, i) = v(i);
				}
				return m;
			};


			auto gyr_scale_m = v2m(state.block(18, 0, 3, 1));

			Eigen::Vector3d gyr =
					(gyr_scale_m * input.block(3, 0, 3, 1) + state.block(12, 0, 3, 1)) *
					time_interval_;
//            std::cout << time_interval_ << std::endl;
			assert(time_interval_ > 0.0 && time_interval_ < 0.1);

			if (input.block(3, 0, 3, 1).norm() > 1e-10) {
				rotation = rotation * Sophus::SO3d::exp(gyr);
//                rotation = Sophus::SO3::exp(gyr) * rotation;

			}

//			auto acc_scale = Eigen::Matrix3d::Identity();
			auto acc_scale = v2m(state.block(15, 0, 3, 1));

			Eigen::Vector3d acc = rotation.matrix() * (acc_scale * input.block(0, 0, 3, 1)) +
			                      Eigen::Vector3d(0, 0, local_gravity_) + state.block(9, 0, 3, 1);
//			auto logger_ptr_ = AWF::AlgorithmLogger::getInstance();
//			logger_ptr_->addPlotEvent("imu_update", "src_acc", input.block(0, 0, 3, 1));
//			logger_ptr_->addPlotEvent("imu_update", "acc", acc);
//			logger_ptr_->addPlotEvent("imu_update", "gyr_src", input.block(3, 0, 3, 1));
//			logger_ptr_->addPlotEvent("imu_update", "gyr", gyr);


//            std::cout << "acc:" << acc.transpose() << std::endl;

			out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
			                              state.block(3, 0, 3, 1) * time_interval_;
//                                          + 0.5 * acc * time_interval_ * time_interval_;
			out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) +
			                              acc * time_interval_;
			out_state.block(6, 0, 3, 1) = rotation.log();
			out_state.block(9, 0, 12, 1) = state.block(9, 0, 12, 1);

			return out_state;

		}

	};


	class FullImuUpdateFunction :
			public ImuUpdateFunction {
	public:
		/**
		 *
		 * @param so3
		 * @param time_interval
		 * @param local_gravity
		 */
		FullImuUpdateFunction(Sophus::SO3d so3,
		                      double time_interval,
		                      double local_gravity) :
				ImuUpdateFunction(15,
				                  time_interval,
				                  local_gravity) {
			epsilon_ = 1e-8;
			rbn_ = so3;

		}


		/**
		 * Core
		 * @param state
		 * @param input
		 * @return
		 */
		Eigen::MatrixXd compute(Eigen::MatrixXd state,
		                        Eigen::MatrixXd input) {
			assert(state.row() == 15);
			Eigen::MatrixXd out_state(15, 1);

			auto rotation = Sophus::SO3d::exp(state.block(6, 0, 3, 1));

			Eigen::Vector3d gyr = (input.block(3, 0, 3, 1) + state.block(12, 0, 3, 1)) * time_interval_;
//			Eigen::Vector3d gyr = (input.block(3, 0, 3, 1)) * time_interval_;
//            std::cout << time_interval_ << std::endl;
			assert(time_interval_ > 0.0 && time_interval_ < 0.1);

			if (input.block(3, 0, 3, 1).norm() > 1e-8) {
				rotation = rotation * Sophus::SO3d::exp(gyr);
//				rotation = Sophus::SO3d::exp(gyr) * rotation;

			}

			Eigen::Vector3d acc = rotation.matrix() * (input.block(0, 0, 3, 1) + state.block(9, 0, 3, 1)) +
			                      Eigen::Vector3d(0, 0, local_gravity_);
//			Eigen::Vector3d acc = rotation.matrix() * input.block(0, 0, 3, 1) +
//			                      Eigen::Vector3d(0, 0, local_gravity_);

//            std::cout << "acc:" << acc.transpose() << std::endl;

			out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
			                              state.block(3, 0, 3, 1) * time_interval_;
//                                          + 0.5 * acc * time_interval_ * time_interval_;

			out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) +
			                              acc * time_interval_;
			out_state.block(6, 0, 3, 1) = rotation.log();
//            rbn_ = rotation;
			out_state.block(9, 0, 6, 1) = state.block(9, 0, 6, 1);

			return out_state;

		}


	};


	class SimpleImuUpdateFunction :
			public ImuUpdateFunction {
	public:
		/**
		 *
		 * @param so3
		 * @param time_interval
		 * @param local_gravity
		 */
		SimpleImuUpdateFunction(Sophus::SO3d so3,
		                        double time_interval,
		                        double local_gravity) :
				ImuUpdateFunction(9,
				                  time_interval,
				                  local_gravity) {
			epsilon_ = 1e-8;
			rbn_ = so3;

		}


		/**
		 * Core
		 * @param state
		 * @param input
		 * @return
		 */
		Eigen::MatrixXd compute(Eigen::MatrixXd state,
		                        Eigen::MatrixXd input) {
			Eigen::MatrixXd out_state(9, 1);

			auto rotation = Sophus::SO3d::exp(state.block(6, 0, 3, 1));

			Eigen::Vector3d gyr = input.block(3, 0, 3, 1) * time_interval_;
//            std::cout << time_interval_ << std::endl;
			assert(time_interval_ > 0.0 && time_interval_ < 0.1);

			if (input.block(3, 0, 3, 1).norm() > 1e-10) {
				rotation = rotation * Sophus::SO3d::exp(gyr);
//                rotation = Sophus::SO3::exp(gyr) * rotation;

			}

			Eigen::Vector3d acc = rotation.matrix() * input.block(0, 0, 3, 1) +
			                      Eigen::Vector3d(0, 0, local_gravity_);
//            std::cout << "acc:" << acc.transpose() << std::endl;

			out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
			                              state.block(3, 0, 3, 1) * time_interval_;
//                                          + 0.5 * acc * time_interval_ * time_interval_;
			out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) +
			                              acc * time_interval_;
			out_state.block(6, 0, 3, 1) = rotation.log();
//            rbn_ = rotation;

			return out_state;

		}


	};
}


#endif //COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
