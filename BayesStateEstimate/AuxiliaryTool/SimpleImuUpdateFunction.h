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
// Created by steve on 18-3-9.
//

#ifndef COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
#define COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H

#include <AWF.h>
#include "ImuUpdateFunction.h"

#include <BSE.h>
#include "ImuTools.h"

#include <sophus/so3.h>


namespace BSE {
    class SimpleImuUpdateFunction : public ImuUpdateFunction {
    public:
        SimpleImuUpdateFunction(double time_interval, double local_gravity) :
                ImuUpdateFunction(9, time_interval, local_gravity) {
            epsilon_ = 1e-8;

        }

        /**
         * Core
         * @param state
         * @param input
         * @return
         */
        Eigen::MatrixXd compute(Eigen::MatrixXd state, Eigen::MatrixXd input) {
            Eigen::MatrixXd out_state(9, 1);

//            Eigen::Quaterniond rotation_q = BSE::ImuTools::angle2q(state.block(6, 0, 3, 1));
            Sophus::SO3 rotation = Sophus::SO3::exp(state.block(6,0,3,1));
            Eigen::Vector3d gyr = input.block(3, 0, 3, 1) * time_interval_;

//            if (input.block(3, 0, 3, 1).norm() > 1e-6) {
            rotation = rotation * Sophus::SO3::exp(gyr);
//                rotation = Sophus::SO3::exp(gyr) * rotation;

//            }

            Eigen::Vector3d acc = rotation.matrix() * input.block(0, 0, 3, 1) + Eigen::Vector3d(0, 0, 9.8);
//            std::cout << "acc:" << acc.transpose() << std::endl;

            out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) + state.block(3, 0, 3, 1) * time_interval_;
//                                          + 0.5 * acc * time_interval_ * time_interval_;
            out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) + acc * time_interval_;
            out_state.block(6, 0, 3, 1) = rotation.log();
            return out_state;

        }


    };
}


#endif //COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
