/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 17-11-7.
//

#ifndef QUICKFUSING_GRAVITYZ_H
#define QUICKFUSING_GRAVITYZ_H

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>

class GravityZ:
        public g2o::BaseBinaryEdge<2,Eigen::Vector2d,g2o::VertexSE3,g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::Vector3d src_acc_ ;//= Eigen::Vector3d(0,0,1);
    Eigen::Vector3d target_acc_ ;//= Eigen::Vector3d(0,0,1);


    GravityZ(Eigen::Vector3d arc_acc,
    Eigen::Vector3d target_acc);

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    virtual void setMeasurement(const Eigen::Vector2d &m) {
        _measurement = m;
//        _inverseMeasurement = 10000;
    }

    virtual bool getMeasurementData(Eigen::Vector2d *d) const {
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

};


#endif //QUICKFUSING_GRAVITYZ_H
