//
// Created by steve on 17-4-25.
//

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

class Line2D : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Line2D() {

    }

    virtual bool read(std::istream &is) {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << std::endl;
        return true;
    }

    virtual bool write(std::ostream &os) const {

        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << std::endl;
        return true;
    }

    virtual void setToOriginImpl() {
        std::cerr << __PRETTY_FUNCTION__ << "not implemented yet" << std::endl;
    }

    virtual void oplusImpl(const double *update) {
        std::cout << "IN OplusImpl" << std::endl;
        for (int i(0); i < 2; ++i) {
            _estimate(i) += update[i];
        }
        std::cout << "after oplusImpl" << std::endl;
    }


    virtual bool setEstimateDataImpl(const double *est) {
//        for(int i(0);i<6;++i)
//        {
//            est[i] = _estimate(i);
//        }

        Eigen::Map<const Eigen::Vector2d> _est(est);
        _estimate = _est;
        return true;
    }

    virtual bool getEstimateData(double *est) const {
        Eigen::Map<Eigen::Vector2d> _est(est);
        _est = _estimate;
        return true;
    }

    virtual int estimateDimension() const {
        return 2;
    }


    virtual bool setMinimalEstimateDataImpl(const double *est) {
        for (int i(0); i < 2; ++i) {
            _estimate(i) = est[i];
        }
        return true;
    }

    virtual bool getMinimalEstimateData(double *est) const {
        Eigen::Map<Eigen::Vector2d> v(est);
        v = _estimate;
        return true;
    }


    virtual int minimalEstimateDimension() const {
        return 2;
    }

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_LINE2D_H
