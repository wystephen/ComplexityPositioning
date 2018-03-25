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
    class SimpleImuUpdateFunction :
            public ImuUpdateFunction {
    public:
        /**
         *
         * @param so3
         * @param time_interval
         * @param local_gravity
         */
        SimpleImuUpdateFunction(Sophus::SO3 so3,
                                double time_interval,
                                double local_gravity) :
                ImuUpdateFunction(9,
                                  time_interval,
                                  local_gravity) {
            epsilon_ = 1e-8;
            rbn = so3;

        }
        Sophus::SO3 rbn=Sophus::SO3(0,0,0);

        /**
         * Core
         * @param state
         * @param input
         * @return
         */
        Eigen::MatrixXd compute(Eigen::MatrixXd state,
                                Eigen::MatrixXd input) {
            Eigen::MatrixXd out_state(9, 1);

            auto rotation = Sophus::SO3::exp(state.block(6,0,3,1));

            Eigen::Vector3d gyr = input.block(3, 0, 3, 1) * time_interval_;
//            std::cout << time_interval_ << std::endl;
            assert(time_interval_ > 0.0 && time_interval_ < 0.1);

            if (input.block(3, 0, 3, 1).norm() > 1e-6) {
                rotation = rotation * Sophus::SO3::exp(gyr);
//                rotation = Sophus::SO3::exp(gyr) * rotation;

            }

            Eigen::Vector3d acc = rotation.matrix() * input.block(0, 0, 3, 1) +
                    Eigen::Vector3d(0, 0, local_gravity_);
//            std::cout << "acc:" << acc.transpose() << std::endl;

            out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) +
                    state.block(3, 0, 3, 1) * time_interval_;
//                                          + 0.5 * acc * time_interval_ * time_interval_;
            out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) +
                    acc * time_interval_;
            out_state.block(6, 0, 3, 1) = rotation.log();
//            rbn = rotation;

            return out_state;

        }


    };
}


#endif //COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
