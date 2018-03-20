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
    namespace ImuTools {

        /**
         * zero velocity detector based on GLRT algorithm.
         * @param u
         * @return
         */
        bool GLRT_Detector(Eigen::MatrixXd u) {
            if (u.cols() == 6 && u.rows() != 6) {
                Eigen::MatrixXd tu = u * 1.0;
//        u = u.transpose();
                u = tu.transpose();
            }
            assert(u.rows() == 6 || "u must be a 6 rows matrix(each col represent acc and gyro at one moement");
            Eigen::Vector3d ya_m;
            double g = 9.8;
            double sigma_a_ = 0.05;
            double sigma_g_ = 0.05 * M_PI / 180.0;
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

        /**
         * Convert eular angle in three axis as quaternion.
         * @param ang
         * @return
         */
        Eigen::Quaterniond angle2q(const Eigen::Vector3d &ang) {
            Eigen::Quaterniond q = Eigen::AngleAxisd(ang(0), Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(ang(1), Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(ang(2), Eigen::Vector3d::UnitZ());

//            Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
//            double bank = ang(0);
//            double attitude = ang(1);
//            double heading = ang(2);
//            double c1 = cos(heading);
//            double s1 = sin(heading);
//            double c2 = cos(attitude);
//            double s2 = sin(attitude);
//            double c3 = cos(bank);
//            double s3 = sin(bank);
//            q.w() = sqrt(1.0 + c1 * c2 + c1 * c3 - s1 * s2 * s3 + c2 * c3) / 2.0;
//            double w4 = (4.0 * q.w());
//            q.x() = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
//            q.y() = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
//            q.z() = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
            return q;
        }

        /**
         * update state(expressed as x)  based on residual (expressed as dx).
         * @param x
         * @param dx
         * @return
         */
        Eigen::Vector3d angleAdd(Eigen::Vector3d x, const Eigen::Vector3d &dx) {
            Eigen::Quaterniond q, dq;
            q = angle2q(x);
            dq = angle2q(dx);
//            q = q * dq;
            q.normalize();
            x = q.toRotationMatrix().eulerAngles(0, 1, 2);


            return x;
        }


    };

};

#endif //COMPLEXITYPOSITIONING_IMUTOOLS_H
