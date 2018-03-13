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
// Created by steve on 18-3-13.
//

#ifndef COMPLEXITYPOSITIONING_MAGGRAVITYMEASUREMENTFUNCTION_H
#define COMPLEXITYPOSITIONING_MAGGRAVITYMEASUREMENTFUNCTION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "MagMeasurementFunction.h"

#include "AWF.h"

class MagGravityMeasurementFunction : public MagMeasurementFunction {
public:
    MagGravityMeasurementFunction() : MagMeasurementFunction() {

    }

    /**
     *  compute
     * @param state
     * @return
     */
    Eigen::MatrixXd compute(Eigen::MatrixXd state) {
        Eigen::Quaterniond q = Eigen::AngleAxisd(state(6, 0), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(state(7, 0), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(state(8, 0), Eigen::Vector3d::UnitZ());

        Eigen::Matrix<double, 6, 1> out;
        out.block(0, 0, 3, 1) = q.inverse() * gravity_nav_;
        out.block(3, 0, 3, 1) = q.inverse() * mag_nav_;
        return out;
    }

    Eigen::Vector3d gravity_nav_ = Eigen::Vector3d(0, 0, 9.81);

    const Eigen::Vector3d &getGravity_nav_() const {
        return gravity_nav_;
    }

    void setGravity_nav_(const Eigen::Vector3d &gravity_nav_) {
        MagGravityMeasurementFunction::gravity_nav_ = gravity_nav_;
    }


};


#endif //COMPLEXITYPOSITIONING_MAGGRAVITYMEASUREMENTFUNCTION_H
