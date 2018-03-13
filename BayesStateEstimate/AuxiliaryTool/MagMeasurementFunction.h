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
// Created by steve on 18-3-12.
//

#ifndef COMPLEXITYPOSITIONING_MAGMEASUREMENTFUNCTION_H
#define COMPLEXITYPOSITIONING_MAGMEASUREMENTFUNCTION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "AWF.h"

class MagMeasurementFunction : public AWF::FunctionAbstract {
public:
    MagMeasurementFunction() : FunctionAbstract(3, 1) {
//        mag_nav_ = mag_in_navigation_frame / mag_in_navigation_frame.norm();
//        std::cout << "mag nav:"
//                  << mag_nav_ << std::endl;

    }

    Eigen::MatrixXd compute(Eigen::MatrixXd state) {
        Eigen::Quaterniond q = Eigen::AngleAxisd(state(6, 0), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(state(7, 0), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(state(8, 0), Eigen::Vector3d::UnitZ());
        return q.inverse() * mag_nav_;


    }

    Eigen::MatrixXd operator()(std::vector<Eigen::MatrixXd> in_vec) {
        return compute(in_vec[0]);
    }

    std::vector<Eigen::MatrixXd> derivative(Eigen::MatrixXd in1) {

        return d(compress(in1));
    }

//    std::vector<Eigen::MatrixXd> minimize_error()

    std::vector<Eigen::MatrixXd> compress(Eigen::MatrixXd input_state) {
        std::vector<Eigen::MatrixXd> t = {};
        t.push_back(input_state);
        return t;
    }


    Eigen::Vector3d mag_nav_ = Eigen::Vector3d(0, 0, 1.0);

//    const Eigen::Vector3d &getMag_nav_() const {
//        return mag_nav_;
//    }

    void setMag_nav(const Eigen::Vector3d &mag_nav_) {
        MagMeasurementFunction::mag_nav_ = mag_nav_ / mag_nav_.norm();
    }

};


#endif //COMPLEXITYPOSITIONING_MAGMEASUREMENTFUNCTION_H
