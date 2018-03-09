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

#ifndef COMPLEXITYPOSITIONING_IMUUPDATEFUNCTION_H
#define COMPLEXITYPOSITIONING_IMUUPDATEFUNCTION_H

#include "AWF.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

class ImuUpdateFunction : public AWF::FunctionAbstract<9, 2> {
public:
    ImuUpdateFunction(Eigen::Quaterniond q) : FunctionAbstract() {

        rotation_q = q;
    }

    /**
     * compress input to std::vector<Eigen::MatrixXd> which is input of super class(FunctionAbstract).
     * @param state
     * @param input
     * @return
     */
    std::vector<Eigen::MatrixXd> compress(Eigen::MatrixXd state, Eigen::MatrixXd input) {
        std::vector<Eigen::MatrixXd> para_vec = {};
        para_vec.push_back(state);
        para_vec.push_back(input);

        return para_vec;

    }


    Eigen::Quaterniond rotation_q = Eigen::Quaterniond::Identity();


};


#endif //COMPLEXITYPOSITIONING_IMUUPDATEFUNCTION_H
