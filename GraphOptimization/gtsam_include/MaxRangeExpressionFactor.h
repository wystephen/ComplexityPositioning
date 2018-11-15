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
// Created by steve on 11/14/18.
//

#ifndef COMPLEXITYPOSITIONING_MAXRANGEEXPRESSIONFACTOR_H
#define COMPLEXITYPOSITIONING_MAXRANGEEXPRESSIONFACTOR_H

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/geometry/Pose3.h>


namespace gtsam {

	/**
	 * @brief return distance - threshold, or 0 when distance < threshold.
	 */
	struct MaxRange {

		typedef Vector1 result_type;
		double threshold_ = 1.5; // TODO: read threshold from MaxRangeExpressionFactor.

		/**
		 * @brief return 0.0 if distance smaller than threshold_.
		 * @param x1
		 * @param x2
		 * @param H1 Matrix<double,1,6> Jacobian matrix of x1
		 * @param H2 Matrix<double,1,6> Jacobian matrix of x2
		 * @return
		 */
		result_type operator()(const Pose3 &x1,
		                       const Pose3 &x2,
		                       OptionalJacobian<1, 6> H1,
		                       OptionalJacobian<1, 6> H2) {

			double dis = x1.range(x2, H1, H2);
			if (dis < threshold_) {
				// set return value to zero and Jacobian matrix also should be zero vector.

				if (H1) {
					*H1 = *H1 * 0.0;
				}

				if (H2) {
					*H2 = *H2 * 0.0;
				}
				return (Vector(1) << 0.0).finished();
			} else {
//				if (H1) {
//					*H1 = exp(dis - threshold_)*dis * *H1;
//				}

//				if (H2) {
//					*H2 = exp(dis - threshold_)*dis * *H2;
//				}
//				return (Vector(1) << exp(dis - threshold_) - 1).finished();
				return (Vector(1) << (dis-threshold_)).finished();

			}
		}
	};


	/**
	 * @brief
	 */
	class MaxRangeExpressionFactor : public ExpressionFactor2<Vector1, Pose3, Pose3> {
	private:
		typedef MaxRangeExpressionFactor This;
		typedef ExpressionFactor2 <Vector1, Pose3, Pose3> Base;

		double threshold_ = 1.0;

	public:


		MaxRangeExpressionFactor() {}

		MaxRangeExpressionFactor(Key key1, Key key2,
		                         double threshold, const SharedNoiseModel &model)
				: Base(key1, key2, model, (Vector(1) << 0.0).finished()) {
			this->initialize(expression(key1, key2));
		}


		/// @return a deep copy of this factor(VERY IMPORTANT!!!!)
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
					gtsam::NonlinearFactor::shared_ptr(new This(*this)));
		}


		Expression <Vector1> expression(Key key1, Key key2) const {
			Expression<Pose3> a1_(key1);
			Expression<Pose3> a2_(key2);


			return Expression<Vector1>(MaxRange(), a1_, a2_);
		}


		/// print
		void print(const std::string &s = "",
		           const KeyFormatter &kf = DefaultKeyFormatter) const {
			std::cout << s << "MaxRangeFactor" << std::endl;
			Base::print(s, kf);
		};
	};


}


#endif //COMPLEXITYPOSITIONING_MAXRANGEEXPRESSIONFACTOR_H
