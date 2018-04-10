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
// Created by steve on 17-12-20.
//

#ifndef QUICKFUSING_SIMPLEROBUSTDISTANCEEDGE_H
#define QUICKFUSING_SIMPLEROBUSTDISTANCEEDGE_H

#include "SimpleDistanceEdge.h"


class SimpleRobustDistanceEdge : public SimpleDistanceEdge {
public:
    SimpleRobustDistanceEdge() {

    }


    /**
     *
     */
    void computeError() {
        g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
        g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

        double p1[10], p2[10];
        from->getEstimateData(p1);
        to->getEstimateData(p2);

        double dis_2 = (p1[0] - p2[0]) * (p1[0] - p2[0]) +
                       (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                       (p1[2] - p2[2]) * (p1[2] - p2[2]);

        double dis = std::sqrt(dis_2);
        if (dis < low_threshold_) {
            ///  dis < low threshold
            if (error_counter_ > 0) {
                _error(0, 0) = dis;

            } else {

                _error(0, 0) = 0.0;

            }

        } else if (dis < high_threshold_) {
            /// high threshold < dis
            if (error_counter_ > 0) {
                _error(0, 0) = dis;

            } else {

                _error(0, 0) = dis - low_threshold_;
            }


        } else {
            /// low threshold < dis < high threshold
            if (error_counter_ > 0) {

                _error(0, 0) = dis - low_threshold_;
            } else {

                _error(0, 0) = high_threshold_ + std::pow(dis - high_threshold_, 0.5) - low_threshold_;
            }

        }
//        _error(0,0) = dis;
        if (error_counter_ >= 0) {
            error_counter_--;
        }


    }

protected:
    double low_threshold_ = (1.5);
    double high_threshold_ = (2.5);
    double error_counter_ = 50;

public:
    void setLow_threshold(double low_threshold) {
        SimpleRobustDistanceEdge::low_threshold_ = low_threshold;
    }

    void setHigh_threshold(double high_threshold) {
        SimpleRobustDistanceEdge::high_threshold_ = high_threshold;
    }


};

#endif //QUICKFUSING_SIMPLEROBUSTDISTANCEEDGE_H
