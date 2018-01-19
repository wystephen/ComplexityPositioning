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
// Created by steve on 17-11-6.
//

#include "RelativeMagEdge.h"


RelativeMagEdge::RelativeMagEdge(const Eigen::Vector3d &src_mag,
                                 const Eigen::Vector3d &target_mag) {
    src_mag_ = src_mag / src_mag.norm();
    target_mag_ = target_mag / target_mag.norm();
}

bool RelativeMagEdge::read(std::istream &is) {
    return true;
}


bool RelativeMagEdge::write(std::ostream &os) const {
    return os.good();
}

void RelativeMagEdge::computeError() {
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);

    Sophus::SO3 from_so3(p1[3], p1[4], p1[5]);
    Sophus::SO3 to_so3(p2[3], p2[4], p2[5]);

    Eigen::Vector3d tmp_vec = (from_so3.matrix() * src_mag_ -
                               to_so3.matrix() * target_mag_);
//    for(int i(0);i<3;++i)
//    {
//        if(tmp_vec(i)>M_PI/2.0)
//        {
//            tmp_vec(i)-=M_PI/2.0;
//        }
//
//        if(tmp_vec(i) < -M_PI/2.0)
//        {
//            tmp_vec(i) += M_PI/2.0;
//        }
//    }

    _error = tmp_vec.array().pow(2.0);
}

bool RelativeMagEdge::setMeasurementFromState() {
    setMeasurement(Eigen::Vector3d(0, 0, 0));
    return true;
}


void RelativeMagEdge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                      g2o::OptimizableGraph::Vertex *to) {

}