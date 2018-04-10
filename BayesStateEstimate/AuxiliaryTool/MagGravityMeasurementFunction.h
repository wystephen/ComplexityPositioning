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

class MagGravityMeasurementFunction : public AWF::FunctionAbstract {
public:
    MagGravityMeasurementFunction() : FunctionAbstract(6, 1) {

    }

    /**
     *  compute
     * @param state
     * @return
     */
    Eigen::MatrixXd compute(Eigen::MatrixXd state) {

        Eigen::Matrix<double, 6, 1> out;
        Sophus::SO3d rbn = Sophus::SO3d::exp(state.block(6, 0, 3, 1));
        out.block(0, 0, 3, 1) = rbn.inverse().matrix() * gravity_nav_;
        out.block(3, 0, 3, 1) = rbn.inverse().matrix() * mag_nav_;
        return out;
    }


    virtual Eigen::MatrixXd operator()(std::vector<Eigen::MatrixXd> in_vec) {
        return compute(in_vec[0]);
    }

    std::vector<Eigen::MatrixXd> derivative(Eigen::MatrixXd in1) {

        return d(compress(in1));

    }


    std::vector<Eigen::MatrixXd> compress(Eigen::MatrixXd input_state) {
        std::vector<Eigen::MatrixXd> t = {};
        t.push_back(input_state);
        return t;
    }


    Eigen::Vector3d mag_nav_ = Eigen::Vector3d(1.0, 0, 0);
    Eigen::Vector3d gravity_nav_ = Eigen::Vector3d(0, 0, 1.0);

    void setMag_nav(const Eigen::Vector3d &mag_nav_) {
        this->mag_nav_ = mag_nav_ / mag_nav_.norm();

    }


//    const Eigen::Vector3d &getGravity_nav_() const {
//        return gravity_nav_;
//    }

    void setGravity_nav_(const Eigen::Vector3d &gravity_nav_) {
        this->gravity_nav_ = gravity_nav_ / gravity_nav_.norm();
    }
};


#endif //COMPLEXITYPOSITIONING_MAGGRAVITYMEASUREMENTFUNCTION_H
