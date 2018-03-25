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

class ImuUpdateFunction : public AWF::FunctionAbstract {
public:
    ImuUpdateFunction(int out_dim, double time_interval, double local_gravity) :
            FunctionAbstract(out_dim, 2) {

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
        int methond_id = 0
        if (methond_id == 0) {

            return d(compress(state, input));
        } else if (methond_id == 1) {
            Eigen::MatrixXd jac_state, jac_input;
            jac_state.resize(OutDim, state.rows());
            jac_input.resize(OutDim, input.rows());
            jac_state.setZero();
            jac_input.setZero();

            auto tmp_state = state;
            auto tmp_input = input;
            auto original_value = operator()(compress(state, input));

            // jacobian of input
            for (int j(0); j < jac_input.cols(); ++j) {
                tmp_input(j) += epsilon_;
                auto tmp_value = operator()(compress(state, tmp_input));
                auto t_d = tmp_value - original_value;
                jac_input.block(0, j, jac_input.rows(), 1) = t_d / double(epsilon_);
                tmp_input(j) -= epsilon_;

            }

            // jacobian of state
            for (int j(0); j < jac_state.cols(); ++j) {

                double tmp_epsilon = epsilon_;
                if (j < 6) {
                    tmp_state(j) += epsilon_;
                } else {
                    tmp_epsilon = 1e-4;
                    Sophus::SO3 r(state(6), state(7), state(8));
                    Eigen::Vector3d td(0, 0, 0);
                    td(j - 6) += tmp_epsilon;
//                r = r * Sophus::SO3::exp(td);
                    r = Sophus::SO3::exp(td) * r;
                    tmp_state.block(6, 0, 3, 1) = r.log();
                }
                auto tmp_value = operator()(compress(tmp_state, input));
                auto t_d = tmp_value - original_value;
                jac_state.block(0, j, jac_state.rows(), 1) = t_d / double(tmp_epsilon);
                if (j < 6) {
                    tmp_state(j) = state(j);
                } else {
//                Sophus::SO3 r(tmp_state(6),tmp_state(7),tmp_state(8));
//                Eigen::Vector3d td(0,0,0);
//                td(j-6) += epsilon_;
//                r = r * Sophus::SO3::exp(td).inverse();
//                tmp_state.block(6,0,3,1) =
                    tmp_state(j) = state(j);

                }
            }


            return compress(jac_state, jac_input);

        }else if(methond_id == 2){


        }else{
            ERROR_MSG_FLAG("invalid method id :"+ std::to_string(methond_id));
        }


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


//    Eigen::Quaterniond rotation_q = Eigen::Quaterniond::Identity();
    double time_interval_ = 0.0;
    double local_gravity_ = -9.81;


};


#endif //COMPLEXITYPOSITIONING_IMUUPDATEFUNCTION_H
