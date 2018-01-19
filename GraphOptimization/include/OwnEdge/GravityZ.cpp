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
// Created by steve on 17-11-7.
//

#include <sophus/so3.h>
#include "GravityZ.h"

GravityZ::GravityZ(Eigen::Vector3d src_acc,
                   Eigen::Vector3d target_acc){
    src_acc_ = src_acc/src_acc.norm();
    target_acc_ = target_acc/target_acc.norm();
//    return true;
}

bool GravityZ::read(std::istream &is){
    return true;
}


bool GravityZ::write(std::ostream &os) const {
    return os.good();
}


/**
 *
 */
void GravityZ::computeError() {
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);

    Sophus::SO3 from_so3(p1[3],p1[4],p1[5]);
    Sophus::SO3 to_so3(p2[3],p2[4],p2[5]);

//    _error(0) = (from_so3.matrix() * src_acc_  + Eigen::Vector3d(0,0,1.0)).norm();
//    _error(1) = (to_so3.matrix() * target_acc_ + Eigen::Vector3d(0,0,1.0)).norm();
    _error(0) = (from_so3.matrix() * src_acc_ -
    to_so3.matrix() * target_acc_).norm();
    _error(1) = _error(0);

}


bool GravityZ::setMeasurementFromState() {
    setMeasurement(Eigen::Vector2d(0.0,0.0));
    return true;
}


void GravityZ::initialEstimate(const g2o::OptimizableGraph::VertexSet &from, g2o::OptimizableGraph::Vertex *to) {

}