//
// Created by steve on 17-4-25.
//

#include "DistanceSE3Line3D.h"

DistanceSE3Line3D::DistanceSE3Line3D() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexLine3D>(),
        tmp_log("tmp_log_for_distance_se3.txt") {
    information().setIdentity();
    _information(0, 0) = 10.0f;
    tmp_log << _information << std::endl;
}

bool DistanceSE3Line3D::read(std::istream &is) {
    return true;
}


bool DistanceSE3Line3D::write(std::ostream &os) const {
    return true;
}


void DistanceSE3Line3D::computeError() {
    /// Wait
    tmp_log << "first code " << std::endl;

    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
    g2o::VertexLine3D *to = static_cast<g2o::VertexLine3D *>(_vertices[1]);
    double p[10] = {0};
    from->getEstimateData(p);

    auto line = to->estimate();
    line.normalize();
    auto direct = line.d();
    auto npoint = line.d().cross(line.w());

    Eigen::Vector3d pose(p[0], p[1], p[2]);

    auto u = pose - npoint;

//    if (direct.norm() < 1e-15) {
//        direct *= 100000;
//    }
//    if()
    tmp_log << "pose:" << pose << "direct :" << direct <<
            " npoint :" << npoint << " u cross direct " << u.cross(direct) << std::endl;

    if (direct.norm() > 1e-10) {

        _error(0, 0) = double((u.cross(direct)).norm() / (direct.norm()));
    } else {
        _error(0, 0) = double(u.norm());
    }

//    if (std::isnan(_error(0, 0))) {
//        _error(0, 0) = 100000;
//    }

//    tmp_log << "error size :" << _error << std::endl;
//    _error(0,0) = 0.0;

}


bool DistanceSE3Line3D::setMeasurementFromState() {
    setMeasurement(0.0f);
    return true;
}


void DistanceSE3Line3D::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                        g2o::OptimizableGraph::Vertex *to) {

}

