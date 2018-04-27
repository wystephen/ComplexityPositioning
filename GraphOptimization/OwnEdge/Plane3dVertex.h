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
// Created by steve on 18-3-7.
//

#ifndef COMPLEXITYPOSITIONING_PLANE3DVERTEX_H
#define COMPLEXITYPOSITIONING_PLANE3DVERTEX_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

class Plane3dVertex: public g2o::BaseVertex<3,Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Plane3dVertex();

    virtual bool setEstimateDataImpl(const double* est){
        for(int i(0);i<3;++i){
            _estimate(i) = est[9];
        }
        return true;
    }

//    virtual

};


#endif //COMPLEXITYPOSITIONING_PLANE3DVERTEX_H
