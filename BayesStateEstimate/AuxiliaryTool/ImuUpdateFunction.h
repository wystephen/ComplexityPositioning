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

#include <AWF.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

template<int OutDim>
class ImuUpdateFunction : public AWF::FunctionAbstract<9, 2> {
public:
    ImuUpdateFunction(Eigen::Quaterniond q, double time_interval, double local_gravity) :
            AWF::FunctionAbstract<9, 2>::FunctionAbstract() {

        rotation_q = q;
        time_interval_ = time_interval;
        local_gravity_ = local_gravity;
    }

    /**
     *
     * @param in_vec
     * @return
     */
    Eigen::MatrixXd operator()(std::vector<Eigen::MatrixXd> in_vec) {
        return compute(in_vec[0], in_vec[1]);
    }

    /**
     *
     * @param state
     * @param input
     * @return
     */
    std::vector<Eigen::MatrixXd> derivative(Eigen::MatrixXd state, Eigen::MatrixXd input) {
        return d(compress(state, input));
    }


    /**
     * state transactoin function. (Core function)
     * @param state
     * @param input
     * @return
     */
    virtual Eigen::MatrixXd compute(Eigen::MatrixXd state, Eigen::MatrixXd input) {
        Eigen::MatrixXd out_state(OutDim, 1);
        return out_state;
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


    /**
     * Convert angle in three axis as quaternion.
     * @param ang
     * @return
     */
    Eigen::Quaterniond angle2q(Eigen::Vector3d ang) {
        Eigen::Quaterniond q = Eigen::AngleAxisd(ang(0), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(ang(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(ang(2), Eigen::Vector3d::UnitZ());
        return q;
    }


    Eigen::Quaterniond rotation_q = Eigen::Quaterniond::Identity();
    double time_interval_ = 0.0;
    double local_gravity_ = -9.81;


};


#endif //COMPLEXITYPOSITIONING_IMUUPDATEFUNCTION_H
