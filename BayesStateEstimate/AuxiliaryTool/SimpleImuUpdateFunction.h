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

namespace BSE {
    class SimpleImuUpdateFunction : public ImuUpdateFunction {
    public:
        SimpleImuUpdateFunction(double time_interval, double local_gravity) :
                ImuUpdateFunction(9,  time_interval, local_gravity) {
            epsilon_ = 1e-7;

        }

        /**
         * Core
         * @param state
         * @param input
         * @return
         */
        Eigen::MatrixXd compute(Eigen::MatrixXd state, Eigen::MatrixXd input) {
            Eigen::MatrixXd out_state(9, 1);

            Eigen::Quaterniond rotation_q = BSE::ImuTools::angle2q(state.block(6, 0, 3, 1));

            rotation_q = rotation_q * BSE::ImuTools::angle2q(input.block(3, 0, 3, 1) * time_interval_);
            rotation_q.normalize();

            Eigen::Vector3d acc = rotation_q.toRotationMatrix() * input.block(0, 0, 3, 1) + Eigen::Vector3d(0, 0, -9.8);

            out_state.block(0, 0, 3, 1) = state.block(0, 0, 3, 1) + state.block(3, 0, 3, 1) * time_interval_;
            out_state.block(3, 0, 3, 1) = state.block(3, 0, 3, 1) + acc * time_interval_;
            out_state.block(6, 0, 3, 1) = rotation_q.toRotationMatrix().eulerAngles(0, 1, 2);
            return out_state;

        }


    };
}


#endif //COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
