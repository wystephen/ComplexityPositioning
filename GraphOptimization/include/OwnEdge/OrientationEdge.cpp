//
// Created by steve on 17-9-5.
//

#include "OrientationEdge.h"

OrientationEdge::OrientationEdge() :g2o::BaseBinaryEdge<3, Sophus::SO3,g2o::VertexSE3,g2o::VertexSE3>(){
    /**
     *
     */

    information().setIdentity();
    _information(0,0) *= 10.0f;
}


bool OrientationEdge::read(std::istream &is) {
    return true;
}

bool OrientationEdge::write(std::ostream &os) const {
    return os.good();
}


void OrientationEdge::computeError() {
    //difference of orientation between two vertex.
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

    double p1[10], p2[10];
    from->getEstimateData(p1);
    to->getEstimateData(p2);

    Sophus::SO3 from_so3(p1[3],p1[4],p1[5]);
    Sophus::SO3 to_so3(p2[3],p2[4],p2[5]);
//    Sophus::SO3 m_so3(_measurement[0],_measurement[1],_measurement[2]);

//    auto phi12 = ((to_so3*from_so3).inverse()*_measurement.inverse()).log();
//    auto phi12 = (to_so3*_measurement*from_so3.inverse()).log();
    auto phi12 =(from_so3.inverse() *  _measurement).log();
//    double delta = std::abs(to_so3.log()(2)-_measurement.log()(2));
//
//    while(delta>2.0*M_PI)
//    {
//        delta -= 2.0*M_PI;
//    }
//    while(1)
//    {
//        if(delta<-M_PI)
//        {
//            delta+=M_PI;
//        }else if(delta>M_PI)
//        {
//            delta -= M_PI;
//        }else{
//            break;
//        }
//    }

    _error(0,0) =  phi12.norm();//...
//    _error(0,0) = std::exp(delta);//...

}

bool OrientationEdge::setMeasurementFromState() {
    setMeasurement(Sophus::SO3(0.0,0.0,0.0));
    return true;
}


void OrientationEdge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                      g2o::OptimizableGraph::Vertex *to) {

}