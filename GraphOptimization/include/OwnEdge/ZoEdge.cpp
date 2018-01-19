//
// Created by steve on 17-3-4.
//

#include "ZoEdge.h"

Z0Edge::Z0Edge() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexSE3>() {
    information().setIdentity();
    _information(0, 0) = 100.0;
}

bool Z0Edge::read(std::istream &is) {
    /**
     * Achive it !!!
     */

    return true;
}

bool Z0Edge::write(std::ostream &os) const {
    //os << "ss" << std::endl;
    return os.good();
}


void Z0Edge::computeError() {
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);
    _error(0, 0) = pow((from->estimate().matrix()(2, 3) - _measurement), 2.0)
                   + pow((to->estimate().matrix()(2, 3) - _measurement), 2.0);
}

bool Z0Edge::setMeasurementFromState() {
    setMeasurement(0);
    return true;

}

//void Z0Edge::linearizeOplus() {
////    std::cout << "linearizeOplus" << std::endl;
//    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
//    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);
//    _jacobianOplusXi(0, 2) = -1.;//*(_error(0,0));
//    _jacobianOplusXj(0, 2) = -1.;//* (_error(0,0));
////    _jacobianOplusXi.setZero();
////    _jacobianOplusXj.setZero();
////    std::cout << " after linearizeOplus" << std::endl;
//}


void Z0Edge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                             g2o::OptimizableGraph::Vertex *to) {
    /**
     * Do nothing
     */
}


