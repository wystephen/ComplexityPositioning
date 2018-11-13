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
// Created by steve on 11/13/18.
//

#ifndef COMPLEXITYPOSITIONING_DUALFEETCONSTRAINT_HPP
#define COMPLEXITYPOSITIONING_DUALFEETCONSTRAINT_HPP


#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {
	struct DualFeetConstraint : public NoiseModelFactor2<Pose3, Pose3> {

		typedef boost::shared_ptr<DualFeetConstraint> shared_ptr;


		double threshold_;

		DualFeetConstraint(Key key1, Key key2, double threshold = 1.0, double sigma = 10000) :
				NoiseModelFactor2<Pose3, Pose3>(noiseModel::Constrained::All(1, sigma), key1, key2),
				threshold_(threshold) {

		}

		~DualFeetConstraint() {
			threshold_ = 1000.0;
		}


		/**
		 * @brief Function called by optimizer
		 * @param x
		 * @param H
		 * @return
		 */
		Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const {
			if (this->active(x)) {
				const Pose3 &x1 = x.at<Pose3>(keys_[0]);
				const Pose3 &x2 = x.at<Pose3>(keys_[1]);
				if (H) {
					return evaluateError(x1, x2, (*H)[0], (*H)[1]);
				} else {
					return evaluateError(x1, x2);
				}
			} else {
				return Vector::Zero(this->dim());
			}
		}


		Vector evaluateError(const Pose3 &x1, const Pose3 &x2,
		                     boost::optional<Matrix &> H1 = boost::none,
		                     boost::optional<Matrix &> H2 = boost::none) const {

			double d = pow(pow(x1.x() - x2.x(), 2.0) +
			               pow(x1.y() - x2.y(), 2.0) +
			               pow(x1.z() - x2.z(), 2.0), 0.5);
			Eigen::Matrix<double, 1, 6> J1, J2;

			J1(0, 0) = (x1.x() - x2.x()) / d;
			J1(0, 1) = (x1.y() - x2.y()) / d;
			J1(0, 2) = (x1.z() - x2.z()) / d;
			for (int i(0); i < 3; ++i) {
				J2(0, i) = -1.0 * J1(0, i);
			}
			for (int i(3); i < 6; ++i) {
				J1(0, i) = 0.0;
				J2(0, i) = 0.0;
			}

			J1 = J1.transpose();
			J2 = J2.transpose();
			if( d > threshold_){
				// constrained

				if(H1){
					std::cout << "H1 :" << H1->rows() << "-----" << H1->cols() << std::endl;
					*H1 = J1 * 1.0;
				}
				if(H2){
					*H2 = J2 * 1.0;
				}
				return (Vector(1) << (d-threshold_)).finished();

			}else{
				if(H1){
					*H1 = J1 * 0.0;
				}
				if(H2){
					*H2 = J2 * 0.0;
				}

				return (Vector(1) << 0.0).finished();


			}

		}


	private:
		/** Serialization function */
		friend class boost::serialization::access;

		template<class ARCHIVE>
		void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
			                                    boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(threshold_);
		}


	};
}

#endif //COMPLEXITYPOSITIONING_DUALFEETCONSTRAINT_HPP
