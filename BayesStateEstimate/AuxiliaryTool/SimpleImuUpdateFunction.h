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

#include "ImuUpdateFunction.h"
class SimpleImuUpdateFunction:public ImuUpdateFunction<9> {
    SimpleImuUpdateFunction(Eigen::Quaternion q,double time_interval):
            ImuUpdateFunction(q,time_interval){

    }

    /**
     * Core
     * @param state
     * @param input
     * @return
     */
    Eigen::MatrixXd compute(Eigen::MatrixXd state, Eigen::MatrixXd input){
        Eigen::MatrixXd out_state(9,1);



    }


};


#endif //COMPLEXITYPOSITIONING_SIMPLEIMUUPDATEFUNCTION_H
