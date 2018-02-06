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

#ifndef COMPLEXITYPOSITIONING_IMUTOOLS_H
#define COMPLEXITYPOSITIONING_IMUTOOLS_H


#include <iostream>

#include <Eigen/Dense>

namespace BSE {
    class ImuTools {
    public:

        bool GLRT_Detector(Eigen::MatrixXd u) {
            if (u.cols() == 6 && u.rows() != 6) {
                Eigen::MatrixXd tu = u * 1.0;
//        u = u.transpose();
                u = tu.transpose();
            }
            assert(u.rows() == 6 || "u must be a 6 rows matrix(each col represent acc and gyro at one moement");
            Eigen::Vector3d ya_m;
            double g = 9.8;
            double sigma_a_ = 0.01;
            double sigma_g_ = 0.1 * M_PI / 180.0;
            double ZeroDetectorWindowSize_ = u.rows();
            double gamma_ = 200;

            double T(0.0);

            for (int i(0); i < 3; ++i) {
                ya_m(i) = u.block(i, 0, 1, u.cols()).mean();
            }

            Eigen::Vector3d tmp;

            for (int i(0); i < u.cols(); ++i) {

                tmp = u.block(0, i, 3, 1) - g * ya_m / ya_m.norm();
                if (std::isnan(tmp.sum())) {
                    std::cout << "nan at tmp in " << __FUNCTION__ << ":"
                              << __FILE__ << ":" << __LINE__ << std::endl;
                }


                T += (u.block(3, i, 3, 1).transpose() * u.block(3, i, 3, 1) / sigma_g_ +
                      tmp.transpose() * tmp / sigma_a_).sum();


            }


            T = T / double(ZeroDetectorWindowSize_);

            if (T < gamma_) {
                return true;

            } else {

                return false;
            }
        }

        /**
 *  process the imu data according to the typical sensor model
 * @param imu_data after preprocess time[s] acc_(x,y,z)[m*s^-2] gyr_(x,y,z)[rad*s^-1] mag_(x,y,z) pressure
 */
        void processImuData(Eigen::MatrixXd &imu_data) {
            Eigen::MatrixXd tmp_data(imu_data);

            int row(imu_data.rows());
            int col(imu_data.cols());
            imu_data.resize(row, 1 + 3 + 3 + 3 + 1);//
            // time
            imu_data.block(0, 0, row, 1) = tmp_data.block(0, 1, row, 1) * 1.0;
            imu_data.block(0, 1, row, 3) = tmp_data.block(0, 2, row, 3) * 9.81;
            imu_data.block(0, 4, row, 3) = tmp_data.block(0, 5, row, 3) * (M_PI / 180.0);
            imu_data.block(0, 7, row, 3) = tmp_data.block(0, 8, row, 3) * 1.0;
            return;
        }
    };

}


#endif //COMPLEXITYPOSITIONING_IMUTOOLS_H
