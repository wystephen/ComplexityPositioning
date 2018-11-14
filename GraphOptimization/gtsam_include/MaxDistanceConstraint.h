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
// Created by steve on 11/11/18.
//

#ifndef COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H
#define COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H

#include <Eigen/Eigen>

#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {
	struct MaxDistanceConstraint : public BoundingConstraint2<Pose3, Pose3> {


		explicit MaxDistanceConstraint(Key key1,
		                               Key key2,
		                               double threshold = 1.5,
		                               bool isGreaterThan = false,
		                               double mu = 1000.0) :
				BoundingConstraint2(key1, key2, threshold, isGreaterThan, mu) {

		}


		/**
		 * @brief Distance between two pose. Note:Const.....!!!!
		 * @param x1: pose3
		 * @param x2: pose3
		 * @param H1: jacobian of x1
		 * @param H2: jacobian of x2
		 * @return
		 */
		double value(const Pose3 &x1, const Pose3 &x2,
		             boost::optional<Matrix &> H1 = boost::none,
		             boost::optional<Matrix &> H2 = boost::none) const {
			if (H1 && H2) {
				std::cout << "H1 size:" << H1->rows() << "x" << H1->cols() << std::endl;
				std::cout << "H2 size:" << H2->rows() << "x" << H2->cols() << std::endl;
			}

			double d = x1.range(x2, H1, H2);
//			std::cout << "distance :" << d
//			<< "  ref distance:" << norm_2((x1.matrix().block(0,3,3,1)-x2.matrix().block(0,3,3,1))) << std::endl;
			if (H1 && H2) {
				std::cout << "H1 size:" << H1->rows() << "x" << H1->cols() << std::endl;
				std::cout << "H2 size:" << H2->rows() << "x" << H2->cols() << std::endl;
			}

			return d;


		}

		bool equals(const NonlinearFactor &f, double tol = 1e-9) const {
			return false;

		}




//		~MaxDistanceConstraint() {
//			std::cout << "Max Distance Constraint  deleted" << std::endl;
//
//		}
	private:

		/** Serialization function */
		friend class boost::serialization::access;

		template<class ARCHIVE>
		void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
			                                    boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(threshold_);
			ar & BOOST_SERIALIZATION_NVP(isGreaterThan_);
		}

	};
};


#endif //COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H
