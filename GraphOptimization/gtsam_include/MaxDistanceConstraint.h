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

		/**
		 * @brief Distance between two pose.
		 * @param x1
		 * @param x2
		 * @param H1
		 * @param H2
		 * @return
		 */
		double value(const X1 &x1, const X2 &x2,
		             Eigen::MatrixXd &H1, Eigen::MatrixXd &H2) {
			double d = pow(pow(x1.x() - x2.x(), 2.0) +
			                   pow(x1.y() - x2.y(), 2.0) +
			                   pow(x1.z() - x2.z(), 2.0), 2.0);

			H1.resize(1,6);
			H2.resize(1,6);

			for(int i(0);i<6;++i){
				H1(0,i) = 0.0;
				H2(0,i) = 0.0;
			}

			H1(0,0) = (x1.x()-x2.x())/d;
			H1(0,1) = (x1.y()-x2.y())/d;
			H1(0,2) = (x1.z()-x2.z())/d;

			H2 = -1.0 * H1;
			return d;

		}

	};
};


#endif //COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H
