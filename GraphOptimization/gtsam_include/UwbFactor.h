//
// Created by steve on 1/5/19.
//

#ifndef COMPLEXITYPOSITIONING_UWBFACTOR_H
#define COMPLEXITYPOSITIONING_UWBFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>


namespace gtsam {
	class UwbFactor : public NoiseModelFactor1<Pose2> {
	private:
		typedef UwbFactor This;
		typedef NoiseModelFactor1 <Pose2> Base;

//		Eigen::VectorXd range_vector_;
//		Eigen::MatrixXd beacon_set_;
		std::vector<double> range_vec_;
		std::vector<Vector2> beacon_set_;
	public:
		UwbFactor() {}

		UwbFactor(Key pose_key, std::vector<double> measured, const SharedNoiseModel &model,
		          std::vector<Vector2> beacon_set) :
				Base(model, pose_key) {
			range_vec_ = measured;
			beacon_set_ = beacon_set;
//			assert(measured.rows() == beacon_set_.rows());
			assert(measured.size() == beacon_set.size());

		}


		/**
		 * @brief
		 * @param pose
		 * @param H
		 * @return
		 */
		Vector evaluateError(const Pose2 &pose, boost::optional<Matrix &> H = boost::none) const {

			int version_id = 0;// 0:standard range constraint
			if (version_id == 0) {
				int dim = range_vec_.size();
				Eigen::MatrixXd error_vector;
				error_vector.resize(dim, 1);
				error_vector.setZero();
				if (H) {
					H->resize(dim, 3);
					H->setZero();
				}
				for (int i = 0; i < range_vec_.size(); ++i) {
//					double ei = pow(pow(pose.x() - beacon_set_[i].x(), 2.0)
//					                + pow(pose.y() - beacon_set_[i].y(), 2.0)
//					                - range_vec_[i] * range_vec_[i], 2.0);
//
					double dis = sqrt(pow(pose.x() - beacon_set_[i].x(), 2.0)
					                  + pow(pose.y() - beacon_set_[i].y(), 2.0));
//
					double ei = (dis - range_vec_[i]);// * (dis - range_vec_[i]);
//					printf("pose x,y:[%f,%f],beacon x,y:[%f,%f]\n",
//					       pose.x(),
//					       pose.y(),
//					       beacon_set_[i].x(),
//					       beacon_set_[i].y());

					if (H) {
						if (dis > 1e-10) {
//							(*H)(i,0) += (4.0 * (pose.x() - beacon_set_[i].x()) * pow(ei, 0.5));
//							(*H)(i,1) += (4.0 * (pose.y() - beacon_set_[i].y()) * pow(ei, 0.5));
							(*H)(i,0) = -1.0 *(pose.x()-beacon_set_[i].x())/dis;
							(*H)(i,1) = -1.0 * (pose.y()-beacon_set_[i].y())/dis;
							(*H)(i,2) = 0.0;
//							(*H)(i, 0) = 2.0 * (dis - range_vec_[i]) * (pose.x() - beacon_set_[i].x()) / dis;
//							(*H)(i, 1) = 2.0 * (dis - range_vec_[i]) * (pose.y() - beacon_set_[i].y()) / dis;

						}


					}


					error_vector(i) = ei;
				}


				return error_vector;
			}

		}


		void print(const std::string &s = "",
		           const KeyFormatter &kf = DefaultKeyFormatter) const {
			std::cout << "UwbFactor" << std::endl;
//			std::cout << " beacon set:" << beacon_set_
//			          << "measurement:" << range_vector_ << std::endl;
		}


	};
}


#endif //COMPLEXITYPOSITIONING_UWBFACTOR_H
