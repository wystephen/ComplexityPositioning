//
// Created by steve on 1/5/19.
//

#include "DistanceEdgeSE2Point2.h"


DistanceEdge2D::DistanceEdge2D() :g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2>() {
	_information.setIdentity();
}



/////




void DistanceEdge2D::computeError() {

	g2o::VertexSE2 *v = static_cast<g2o::VertexSE2 *>(_vertices[0]);

	double p[3];
	v->getEstimateData(p);

	for (int i = 0; i < dis_vec_.size(); ++i) {
		double dis = std::sqrt(pow(p[0] - beacon_set_vec_[i].x(), 2.0)
		                       + pow(p[1] - beacon_set_vec_[i].y(), 2.0));
		_error(0, 0) += std::pow(dis - dis_vec_[i], 2.0);
	}

}