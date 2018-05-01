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

//
class HardConstraintIMU : public g2o::BaseBinaryEdge<1, Eigen::Isometry3d, g2o::VertexSE3, g2o::VertexSE3> {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	HardConstraintIMU() {
		// initial funciton base on memery cpy

	}

	virtual bool read(std::istream &is) {
		return true;
	}

	virtual bool write(std::ostream &os) const {
		return os.good();
	}

	/**
	 * @brief compute error of current system state
	 */
	virtual void computeError() {
		// TODO: 1-dof or 2-dof?

	}

	/**
	 *
	 * @param m
	 */
	virtual void setMeasurement(const double &m) {
//		_measurement = m;
		Eigen::Matrix<double, 6, 1> m_v;
		for (int i(0); i < 6; ++i) {
			m_v(i) = m[i];
		}
		_measurement = g2o::internal::fromVectorMQT(m_v);
	}

	void setMeasurement(Eigen::Isometry3d &m) {
		_measurement = m;
	}

	virtual bool getMeasurementData(double *d) const {
//		*d = ;
//		*d = _measurement.data();

		auto v = g2o::internal::toVectorQT(_measurement);
		for (int i(0); i < 6; ++i) {
			d[i] = v(i);
		}

		return true;
	}


/**
 * ...
 * @return
 */
	virtual int measurementDimension() const {
		return 6;
	}

	virtual bool setMeasurementFromState() {
		return false;
	}

	/**
	 * pure virtual in meaning side actually~
	 * @return
	 */
	virtual double initialEstimatePossible(
			const g2o::OptimizableGraph::VertexSet &/*from*/,
			g2o::OptimizableGraph::Vertex */*to*/) {
//        //TODO:
//        std::cout << __FILE__ << __FUNCTION__
//                  << __LINE__ << "this function not implement" << std::endl;
		return 1.0;
	}


};


#endif //COMPLEXITYPOSITIONING_HARDCONSTRAINTIMU_H
