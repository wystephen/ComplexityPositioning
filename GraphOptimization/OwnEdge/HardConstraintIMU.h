//
// Created by steve on 18-4-29.
//

#ifndef COMPLEXITYPOSITIONING_HARDCONSTRAINTIMU_H
#define COMPLEXITYPOSITIONING_HARDCONSTRAINTIMU_H
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
class HardConstraintIMU:public g2o::BaseBinaryEdge<1 ,double, g2o::VertexSE3, g2o::VertexSE3>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	HardConstraintIMU(){
		// initial funciton base on memery cpy

	}





};


#endif //COMPLEXITYPOSITIONING_HARDCONSTRAINTIMU_H
