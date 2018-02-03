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

        auto uwb_err_function = [&uwb_data_,
                &beacon_set_,
                &uwb_index]
                (Eigen::Vector3d pos) -> double {
            int vaild_counter = 0;
            double sum_err = 0.0;
            for (int i(1); i < uwb_data_.cols(); ++i) {
                if (uwb_data_(0, i) > 0.0) {
                    sum_err += std::abs(uwb_data_(0, i) -
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




    }
}


#endif //COMPLEXITYPOSITIONING_UWBTOOLS_H
