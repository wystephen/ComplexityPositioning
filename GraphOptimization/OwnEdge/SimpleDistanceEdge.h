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
// Created by steve on 17-11-17.
//

#ifndef QUICKFUSING_SIMPLEDISTANCEEDGE_H
#define QUICKFUSING_SIMPLEDISTANCEEDGE_H

#include "DistanceEdge.h"
#include "DistanceEdge.cpp"


class SimpleDistanceEdge : public DistanceEdge {
public:
    SimpleDistanceEdge() {

    }

    virtual void computeError() {
        g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
        g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

        double p1[10], p2[10];
        from->getEstimateData(p1);
        to->getEstimateData(p2);

        double dis_2 = (p1[0] - p2[0]) * (p1[0] - p2[0]) +
                       (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                       (p1[2] - p2[2]) * (p1[2] - p2[2]);
//        std::cout << dis_2 << std::endl;
        _error(0, 0) = dis_2;

    }

};


#endif //QUICKFUSING_SIMPLEDISTANCEEDGE_H
