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

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H


class Line3D : public g2o::BaseVertex<6, Eigen::Matrix<double, 1, 6>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Line3D() {

    }

    virtual bool read(std::istream &is) {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
        return true;
    }

    virtual bool write(std::ostream &os) {

        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
        return true;
    }

    virtual void setToOriginImpl() {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << endl;
    }

    virtual void oplusImpl(const double *update) {
        for (int i(0); i < 6; ++i) {
            _estimate(i) += update[i];
        }
    }


    virtual bool setEstimateDataImpl(const double *est) {
//        for(int i(0);i<6;++i)
//        {
//            est[i] = _estimate(i);
//        }

        Eigen::Map<const T> _est(est);
        _estimate = _est;
        return true;
    }

    virtual bool getEstimateData(double *est) const {
        Eigen::Map<T> _est(est);
        _est = _estimate;
        return true;
    }

    virtual int estimateDimension() const {
        return 6;
    }


    virtual bool setMinimalEstimateDataImpl(const double *est) {
        _estimate = Eigen::Map<T>(est);
        return true;
    }

    virtual bool getMinimalEstimateData(double *est) const {
        Eigen::Map<T> v(est);
        v = _estimate;
        return true;
    }


    virtual int minimalEstimateDimension() const {
        return 6;
    }


};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE3D_H
