//
// Created by steve on 17-4-25.
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


#include "OwnEdge/Line2D.h"
#include "OwnEdge/Line2D.cpp"


#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H


class Point2Line2D : public g2o::BaseBinaryEdge<1, double, Line2D, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Point2Line2D() :
            g2o::BaseBinaryEdge<1, double, Line2D, g2o::VertexSE3>() {
        information().setIdentity();
        _information(0, 0) = 10.0f;

    }

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) const {
        return true;
    }


    void computeError();

    virtual void setMeasurement(const double &m) {
        _measurement = m;
    }

    virtual bool getMeasurementData(double *d) const {
        *d = _measurement;
        return true;
    }

//    void linearizeOplus();

    virtual int measurementDimension() const {
        return 1;
    }

    virtual bool setMeasurementFromState();

    virtual double initialEstimatePossible(
            const g2o::OptimizableGraph::VertexSet &/*from*/,
            g2o::OptimizableGraph::Vertex */*to*/) {
//        //TODO:
//        std::cout << __FILE__ << __FUNCTION__
//                  << __LINE__ << "this function not implement" << std::endl;
        return 1.0;
    }


    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                 g2o::OptimizableGraph::Vertex *to);

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINT2LINE2D_H
