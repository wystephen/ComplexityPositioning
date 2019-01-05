//
// Created by steve on 1/5/19.
//

#include "DistanceEdgeSE2Point2.h"


DistanceEdge2D::DistanceEdge2D() :g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2>() {
	_information.setIdentity();
}



/////



//
//void DistanceEdge2D::computeError() {
//
//
//}