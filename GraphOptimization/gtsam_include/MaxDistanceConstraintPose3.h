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
	struct MaxDistanceConstraintPose3 : public BoundingConstraint2<Pose3, Pose3> {
		typedef MaxDistanceConstraintPose3 This;
		typedef boost::shared_ptr<MaxDistanceConstraintPose3> shared_ptr;
		typedef BoundingConstraint2<Pose3, Pose3> Base;


		explicit MaxDistanceConstraintPose3(Key key1,
		                                    Key key2,
		                                    double threshold = 1.5,
		                                    bool isGreaterThan = false,
		                                    double mu = 1000.0) :
				Base(key1, key2, threshold, isGreaterThan, mu) {

		}


		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
					gtsam::NonlinearFactor::shared_ptr(new This(*this)));
		}


		/**
		 * @brief Distance between two pose. Note:Const.....!!!!
		 * @param x1: pose3
		 * @param x2: pose3
		 * @param H1: jacobian of x1
		 * @param H2: jacobian of x2
		 * @return
		 */
		virtual double value(const Pose3 &x1, const Pose3 &x2,
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


		~MaxDistanceConstraintPose3() {
//			std::cout << "Max Distance Constraint  deleted" << std::endl;
		}

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


	/**
     * Trait for distance constraints to provide distance
     * @tparam T1 is a Lie value for which distance functions exist
     * @tparam T2 is a Lie value for which distance functions exist
     * @param a is the first Lie element
     * @param b is the second Lie element
     * @return a scalar distance between values
     */
	template<class T1, class T2>
	double range_trait(const T1 &a, const T2 &b) { return a.rang(b); }

	/**
	 * Binary inequality constraint forcing the range between points
	 * to be less than or equal to a bound
	 * @tparam VALUES is the variable set for the graph
	 * @tparam KEY is the type of the keys for the variables constrained
	 */
	template<class VALUE>
	struct MaxDistanceConstraint : public BoundingConstraint2<VALUE, VALUE> {
		typedef BoundingConstraint2<VALUE, VALUE> Base;  ///< Base class for factor
		typedef MaxDistanceConstraint<VALUE> This;  ///< This class for factor
		typedef VALUE Point; ///< Type of variable constrained

		virtual ~MaxDistanceConstraint() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
					gtsam::NonlinearFactor::shared_ptr(new This(*this)));
		}

		/**
		 * Primary constructor for factor
		 * @param key1 is the first variable key
		 * @param key2 is the second variable key
		 * @param range_bound is the maximum range allowed between the variables
		 * @param mu is the gain for the penalty function
		 */
		MaxDistanceConstraint(Key key1, Key key2, double range_bound, double mu = 1000.0) :
				Base(key1, key2, range_bound, false, mu) {}

		/**
		 * computes the range with derivatives
		 * @param x1 is the first variable value
		 * @param x2 is the second variable value
		 * @param H1 is an optional Jacobian in x1
		 * @param H2 is an optional Jacobian in x2
		 * @return the distance between the variables
		 */
		virtual double value(const Point &x1, const Point &x2,
		                     boost::optional<Matrix &> H1 = boost::none,
		                     boost::optional<Matrix &> H2 = boost::none) const {
//			if (H1) *H1 = numericalDerivative21(range_trait<Point, Point>, x1, x2, 1e-5);
//			if (H1) *H2 = numericalDerivative22(range_trait<Point, Point>, x1, x2, 1e-5);

//			return range_trait(x1, x2);
			return x1.range(x2,H1,H2);
		}
	};

};


#endif //COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H
