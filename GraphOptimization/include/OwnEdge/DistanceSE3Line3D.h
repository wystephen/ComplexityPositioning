//
// Created by steve on 17-4-25.
//

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_DISTANCESE3LINE3D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_DISTANCESE3LINE3D_H


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"


#include "MyError.h"

class DistanceSE3Line3D :
        public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexLine3D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DistanceSE3Line3D();

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    /**
     * distance
     * @param m
     */
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

//    virtual double initialEstimatePossible(
//            const g2o::OptimizableGraph::VertexSet &/*from*/,
//            g2o::OptimizableGraph::Vertex */*to*/) {
////        //TODO:
//        std::cout << __FILE__ << __FUNCTION__
//                  << __LINE__ << "this function not implement" << std::endl;
//        return 1.0;
//    }


    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                 g2o::OptimizableGraph::Vertex *to);

//    void linearizeOplus(){
////        std::cout << _jacobianOplusXi.size();
//        _jacobianOplusXi.setZero();
//        _jacobianOplusXj.setZero();
//
//    }

    std::ofstream tmp_log;

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_DISTANCESE3LINE3D_H
