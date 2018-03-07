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
// Created by steve on 18-2-3.
//

#ifndef COMPLEXITYPOSITIONING_UWBTOOLS_H
#define COMPLEXITYPOSITIONING_UWBTOOLS_H

#include <Eigen/Dense>

namespace BSE {
    class UwbTools {
    public:
        UwbTools(Eigen::MatrixXd uwb_data, Eigen::MatrixXd beacon_set_data) :
                uwb_data_(uwb_data), beacon_set_(beacon_set_data) {


        }


        Eigen::MatrixXd uwb_data_;
        Eigen::MatrixXd beacon_set_;
        int uwb_index = 0;

        /**
         * check the data if it is available.
         * @return wether the data is available.
         */
        bool checkData() {
            if (uwb_data_.rows() > 0 && beacon_set_.rows() > 0) {
                if (uwb_data_.cols() - 1 == beacon_set_.rows()) {

                    return true;
                } else {
                    std::cout << uwb_data_.rows() << "x" << uwb_data_.cols() << std::endl;
                    std::cout << beacon_set_.rows() << "x" << beacon_set_.cols() << std::endl;

                    return false;
                }

            } else {
                std::cout << __FUNCTION__
                          << ":"
                          << __LINE__
                          << "uwb data or beacon set is zeros"
                          << std::endl;
                return false;
            }
        }

        /**
         * return the orientation of the function.
         */
        std::function<double(Eigen::MatrixXd &)> computeInitialOri = [&]
                (Eigen::MatrixXd &trace_data) -> double {
            if (!checkData()) {
                std::cout << __FILE__ << ":" << __LINE__ << std::endl;
                return 0.0;
            }

            int i(0);
            int j(0);
            // TODO: soft RANSAC for choice right orientation.
            for (; j < trace_data.rows(); ++j) {
                if ((trace_data.block(j, 0, 1, 3) - trace_data.block(i, 0, 1, 3)).norm() > 3.0) {
                    return std::atan2(trace_data(j, 1) - trace_data(i, 1),
                                      trace_data(j, 0) - trace_data(i, 0));

                }
            }


        };


        /**
         * err function.
         * @param pose
         * return error according to beacon set.
         */
        std::function<double(Eigen::Vector3d)> uwb_err_function = [&]
                (Eigen::Vector3d pos) -> double {
            if (!checkData()) {
                return 0.0;
            }
            int vaild_counter = 0;
            double sum_err = 0.0;
            for (int i(1); i < uwb_data_.cols(); ++i) {
                if (uwb_data_(uwb_index, i) > 0.0) {
                    sum_err += std::abs(uwb_data_(uwb_index, i) -
                                        (pos - beacon_set_.block(i - 1, 0, 1, 3).transpose()).norm());
                    vaild_counter++;
                }

            }
            if (vaild_counter > 0) {

                return sum_err / double(vaild_counter);
            } else {
                return 1000000.0;
            }
        };

        /**
         *
         * @param trace based on
         * return
         */
        std::function<Eigen::MatrixXd()> uwb_position_function = [
                &]() -> Eigen::MatrixXd {
            if (!checkData()) {
                std::cout << __FUNCTION__
                          << ":"
                          << __FILE__
                          << ":"
                          << __LINE__
                          << std::endl;
                return Eigen::Matrix3d::Identity();
            }
            Eigen::MatrixXd trace = Eigen::MatrixXd(uwb_data_.rows(), 4);
            Eigen::Vector3d initial_pos(0, 0, 0);
            for (int i(0); i < trace.rows(); ++i) {
                uwb_index = i;
                double last_uwb_err = 10000000000.0;
                int ite_times = 0;

                double step_length = 0.00001;
                double update_rate = 0.15;

                while (ite_times < 1000 ) {
                    last_uwb_err = uwb_err_function(initial_pos);
                    Eigen::Vector3d tmp_gradient(0, 0, 0);
                    for (int i(0); i < 3; ++i) {
                        auto t_v = initial_pos;
                        t_v(i) += step_length;
                        tmp_gradient(i) = (uwb_err_function(t_v) - last_uwb_err) / step_length;
                    }
                    initial_pos -= tmp_gradient * update_rate;


//                    std::cout << "last err:" << last_uwb_err << std::endl;
                    ite_times++;

                }


                trace.block(i, 0, 1, 3) = initial_pos.transpose();
                trace(i, 3) = uwb_err_function(initial_pos);


            }
            return trace;

        };


    };
}


#endif //COMPLEXITYPOSITIONING_UWBTOOLS_H
