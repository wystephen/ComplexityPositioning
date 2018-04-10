//
// Created by steve on 17-3-4.
//

#ifndef ARSLAM_Z_ZERO_EDGE_H
#define ARSLAM_Z_ZERO_EDGE_H

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"


//G2O_USE_TYPE_GROUP(slam3d);

class Z0Edge :
        public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Z0Edge();

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    virtual void setMeasurement(const double &m) {
        _measurement = m;
//        _inverseMeasurement = 10000;
    }

    virtual bool getMeasurementData(double *d) const {
        *d = _measurement;
        return true;
    }

//    void linearizeOplus();

    virtual int measurementDimension() const { return 1; }

    virtual bool setMeasurementFromState();

    virtual double initialEstimatePossible(
            const g2o::OptimizableGraph::VertexSet &,
            g2o::OptimizableGraph::Vertex */*to*/) {
        return 1.0;
    }

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                 g2o::OptimizableGraph::Vertex *to);



// Load g2o file and add new tag .....


};


#endif //ARSLAM_Z_ZERO_EDGE_H
