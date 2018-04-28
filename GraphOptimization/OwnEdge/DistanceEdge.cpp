//
// Created by steve on 17-4-15.
//

#include "DistanceEdge.h"

DistanceEdge::DistanceEdge() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexSE3>() {
    /**
     *
     */
    information().setIdentity();
    _information(0, 0) = 10.0f;
}

bool DistanceEdge::read(std::istream &is) {
    /**
     * TODO: ACHIVE IT!!!
     */
    return true;
}

bool DistanceEdge::write(std::ostream &os) const {
    return os.good();
}


int counter = 0;

void DistanceEdge::computeError() {
//    counter ++;
//    std::cout << "compute err L" << std::endl;
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);

    double dis = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                           (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                           (p1[2] - p2[2]) * (p1[2] - p2[2]));


    if (std::isnan(dis)) {
        std::cout << "stop here" << std::endl;
    }
    try {

        _error(0, 0) = std::pow(dis - _measurement, 2.0);
//        if(counter > 1000 && dis-_measurement>3.0){
//            _error(0,0) = 0.0;
//        }
        if (std::isnan(_error(0, 0)) || std::isinf(_error(0, 0))) {
            throw std::bad_cast();
        }
    } catch (std::exception &e) {
        std::cout << e.what() << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__ << std::endl;
    } catch (...) {

        std::cout << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__ << std::endl;
    }


}

bool DistanceEdge::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}


//void DistanceEdge::linearizeOplus() {
//    /**
//     *  i from j to .
//     *  vertexSE3 (x,y,z wx wy wz)...
//     *
//     */
//    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
//    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);
//
//    double p1[10], p2[10];
//    from->getEstimateData(p1);
//    to->getEstimateData(p2);
//
//    _jacobianOplusXi.setZero();
//    _jacobianOplusXj.setZero();
//    double dis = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
//                           (p1[1] - p2[1]) * (p1[1] - p2[1]) +
//                           (p1[2] - p2[2]) * (p1[2] - p2[2]));
//
//    auto value_of=[dis,p1,p2](double *delta_p1,double *delta_p2)->double
//    {
//        double new_dis(0.0);
//        new_dis = std::sqrt((p1[0] - p2[0]+delta_p1[0]-delta_p2[0]) * (p1[0] - p2[0]+delta_p1[0]-delta_p2[0]) +
//                                   (p1[1] - p2[1]+delta_p1[1]-delta_p2[1]) * (p1[1] - p2[1]+delta_p1[1]-delta_p2[1]) +
//                                   (p1[2] - p2[2]+delta_p1[2]-delta_p2[2]) * (p1[2] - p2[2]+delta_p1[2]-delta_p2[2]));
//        return new_dis;
//    };
//
////    double delta_G = 1.0;
////    delta_G = _error(0,0) * (dis-_measurement)/sigma_/sigma_;
////    delta_G = std::log(1/std::sqrt(2*M_PI)/sigma_) * (dis-_measurement)/sigma_/sigma_;
//
//    double delta_offset(_error(0,0)/1000000000.0);
//
//    for (int i(0); i < 3; ++i) {
////        if (std::abs(p1[i] - p2[i]) - _measurement <1e-20) {
////            continue;
////        }
//
//        double dp1[3]={0};
//        double dp2[3]={0};
//
//
//
//        dp1[i] = delta_offset;
//        _jacobianOplusXi(0,i)=-(std::log(NormalPdf(dis,_measurement,sigma_))-std::log(NormalPdf(value_of(dp1,dp2),_measurement,sigma_)))/delta_offset;
//        dp1[i]=0.0;
//        dp2[i] = delta_offset;
//        _jacobianOplusXj(0,i)=-(std::log(NormalPdf(dis,_measurement,sigma_))-std::log(NormalPdf(value_of(dp1,dp2),_measurement,sigma_)))/delta_offset;
//
////        _jacobianOplusXi(0, i) = -(p1[i] - p2[i]) * (dis )*delta_G;
////        _jacobianOplusXj(0, i) = (p1[i] - p2[i])* 2.0 * (dis )*delta_G;
//    }
//
//}


void DistanceEdge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                   g2o::OptimizableGraph::Vertex *to) {

    /**
     * Do nothing
     */
}