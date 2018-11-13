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


		explicit MaxDistanceConstraint(Key key1, Key key2, double threshold = 1.5, bool isGreaterThan = false,
		                               double mu = 1000.0) :
				BoundingConstraint2(key1, key2, threshold, isGreaterThan, mu) {
			int a = 0;

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

			if(H1){
//				std::cout << "H1 is not empty" << std::endl;
				std::cout << H1->rows() << "," << H1->cols() << std::endl;
				*H1 = J1*1.0;
			}

			if(H2){
//				std::cout << "H2 is not empty" << std::endl;
				*H2=J2*1.0;
			}


//			std::cout<< "x1:" << x1.matrix() <<"\n";
//			std::cout << "x2:" << x2.matrix() << "\n";
//			std::cout << "distancce:" << d << std::endl;


//			std::cout << "value:" << d << std::endl;

			return d;

		}


		~MaxDistanceConstraint() {
//			delete threshold_;
		}


	};
};


#endif //COMPLEXITYPOSITIONING_BOUNDINGDISTANCE_H
