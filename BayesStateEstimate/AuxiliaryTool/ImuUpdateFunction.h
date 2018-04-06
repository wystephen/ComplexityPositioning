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
        int methond_id = 2;
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
                if (j < 3) {

                    tmp_input(j) += epsilon_;
                } else {
                    tmp_input(j) += epsilon_ / 180.0 * M_PI;
                }
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
                    tmp_state(j) += epsilon_ / 180.0 * M_PI;

                }
                if (j >= 6 && j < 9) {
//                    while(tmp_state(j) > M_PI){
//                        tmp_state(j) -= 2.0 * M_PI;
//                    }
//                    while(tmp_state(j) < -M_PI){
//                        tmp_state(j) += 2.0 * M_PI;
//                    }
//                    Sophus::SO3 r = Sophus::SO3::exp(tmp_state.block(6, 0, 3, 1));
//                    Eigen::Vector3d so3_diff(0.0, 0.0, 0.0);
//                    so3_diff(j - 6) = epsilon_;
////                    r = Sophus::SO3::exp(so3_diff) * r;
//                    r = r * Sophus::SO3::exp(so3_diff);
//                    tmp_state.block(6, 0, 3, 1) = r.log();
                }
                auto tmp_value = operator()(compress(tmp_state, input));
                auto t_d = tmp_value - original_value;
                jac_state.block(0, j, jac_state.rows(), 1) = t_d / double(tmp_epsilon);

//                tmp_state(j) = state(j);
                tmp_state = state;

            }


            return compress(jac_state, jac_input);

        } else if (methond_id == 2) {
            /**
             *
             */
            auto c_state = compute(state, input);
            Eigen::MatrixXd F(9, 9);
            Eigen::MatrixXd G(9, 6);
            F.setZero();
            G.setZero();

            auto rotation = Sophus::SO3d::exp(c_state.block(6, 0, 3, 1));


            auto f_t = Eigen::Vector3d(input.block(0, 0, 3, 1));
            f_t = rotation.matrix() * f_t;

            Eigen::Matrix3d st;
            st << 0.0, -f_t(2), f_t(1),
                    f_t(2), 0.0, -f_t(0),
                    -f_t(1), f_t(0), 0.0;
            /////////////
            F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
            F.block(3, 6, 3, 3) = st;
            F = F * time_interval_;
            F += Eigen::Matrix<double, 9, 9>::Identity();



            /////////////////
            G.block(3, 0, 3, 3) = rotation.matrix().eval();
            G.block(6, 3, 3, 3) = -1.0 * rotation.matrix().eval();
            G = time_interval_ * G;

            return compress(F, G);


        } else {
            ERROR_MSG_FLAG("invalid method id :" + std::to_string(methond_id));
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
