//
// Created by steve on 17-4-21.
//

#ifndef MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINTONLINE_H
#define MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINTONLINE_H

#include "g2o/config.h"
#include "g2o_types_slam3d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "vertex_se3.h"
#include "vertex_pointxyz.h"

class PointonLine : public g2o::BaseMultiEdge<-1, g2o::VertexSE3> {
protected:
    unsigned int _observedPoints;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointonLine();

    void setDimension(int dimension_) {
        _dimension = dimension_;
        _information.resize(dimension_, dimension_);
        _error.resize(dimension_, 1);
        _measurement.resize(dimension_, 1);

    }

    void setSize(int verteices) {
        resize(verteices);
        _observedPoints = vertices - 1;
        setDimension(_observedPoints * 3);
    }

    virtual void computeError();

    virtual bool read(std::istream &is) {
        return true;
    }

    virtual bool write(std::ostream &os) {
        return true;
    }

    virtual bool setMeasurementFromState() {
        return true;
    }

//    virtual void initialEstimate(const )

};


#endif //MAGNETOMETERFEATUREAUGUMENTATIONLOCATION_POINTONLINE_H
