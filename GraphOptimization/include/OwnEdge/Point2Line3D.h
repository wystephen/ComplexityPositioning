//
// Created by steve on 17-4-21.
//

#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"


#include "OwnEdge/Line3D.h"
#include "OwnEdge/Line3D.cpp"

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE3D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE3D_H


class Point2Line3D : public g2o::BaseBinaryEdge<1, g2o::VertexSE3, Line3D> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


    Point2Line3D() {

    }

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) const {
        return true;
    }


    /**
     * Distance from point to line.
     */
    void computeError() {
        double p[10] = {0};
        double l[10] = {0};
        const auto *Line = static_cast<const Line3D *>(vertices()[0]);
        Line->getEstimateData(l);
        g2o::VertexSE3 *point = static_cast<g2o::VertexSE3 *>(vertices()[1]);
        p = point->getEstimateData(p);


        Eigen::Vector3d A, B, C;
        for (int i(0); i < 3; ++i) {
            A(i) = p[i];
            B(i) = l[i];
            C(i) = l[i + 3];
        }


        if ((B - A).norm() < 1e-10) {
            _error(0, 0) = 1000;
            std::cout << "SOME ERROR WHEN COMPUTE ERRO OF POINT2Line3D" << std::endl;
        }
        _error(0, 0) = (B - A).cross(C - A) / (B - A).norm();

    }

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE3D_H
