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
// Created by steve on 18-3-8.
//

#ifndef COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
#define COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H


#include <sophus/so3.h>
#include <sophus/se3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "KalmanFilterBase.h"
#include "KalmanFilterBase.h"

namespace BSE {


    class KFComplex {
    public:

        KFComplex(Eigen::Matrix<double, 15, 15> init_prob) {
            prob_state_ = init_prob;
            state_x_.setZero();
        }


        bool initial_state(Eigen::MatrixXd imu_data,
                           double initial_ori = 0.0,
                           Eigen::Vector3d initial_pose = Eigen::Vector3d(0, 0, 0)) {



        }


        /**
         * dax day daz : offset of acc measurements.
         * dgx dgy dgz : offset of gyr measurements.
         */
        Eigen::Matrix<double, 15, 1> state_x_;//x y z vx vy vz wx wy wz dax day daz dgx dgy dgz


        Eigen::Matrix<double, 15, 15> prob_state_; // probability of state


        double time_interval_ = 0.005;// time interval

        double local_g_ = -9.81; // local gravity acc.


    };
}


#endif //COMPLEXITYPOSITIONING_IMUWBKFCOMPLEX_H
