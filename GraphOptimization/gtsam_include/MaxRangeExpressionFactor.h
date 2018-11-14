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

	struct MaxRange {

		typedef Vector1 result_type;
		double threshold_ = 1.0;

		result_type operator()(const Pose3 &x1, const Pose3 &x2, OptionalJacobian<1, 6> H1, OptionalJacobian<1, 6> H2) {
			double dis = x1.range(x2, H1, H2);
			if (dis < threshold_) {
				return (Vector(1) << 0.0).finished();
			} else {
				return (Vector(1) << (dis - threshold_)).finished();
			}
		}
	};


	class MaxRangeExpressionFactor : public ExpressionFactor2<Vector1, Pose3, Pose3> {
	private:
		typedef MaxRangeExpressionFactor This;
		typedef ExpressionFactor2<Vector1, Pose3, Pose3> Base;

		double threshold_ = 1.0;

	public:

		MaxRangeExpressionFactor() {}

		MaxRangeExpressionFactor(Key key1, Key key2,
		                         double threshold, const SharedNoiseModel &model)
				: Base(key1, key2, model, (Vector(1) << 0.0).finished()) {

		}


		Expression<Vector1> expression(Key key1, Key key2) const {
			Expression < Pose3 > a1_(key1);
			Expression < Pose3 > a2_(key2);

//			auto MaxRange = [](const Pose3& x1,const Pose3& x2){
//
//			};
//
			return Expression < Vector1 > (MaxRange(), a1_, a2_);
		}


	};


}


#endif //COMPLEXITYPOSITIONING_MAXRANGEEXPRESSIONFACTOR_H
