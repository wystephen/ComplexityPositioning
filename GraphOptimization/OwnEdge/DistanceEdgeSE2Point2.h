//
// Created by steve on 1/5/19.
//

#ifndef COMPLEXITYPOSITIONING_DISTANCEEDGESE2POINT2_H
#define COMPLEXITYPOSITIONING_DISTANCEEDGESE2POINT2_H


#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"


class DistanceEdge2D :
		public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	DistanceEdge2D();

	virtual bool read(std::istream &is) {
		return true;
	}

	virtual bool write(std::ostream &os) const {
		return true;
	}

	virtual void computeError() {

		g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2 *>(_vertices[0]);
		g2o::VertexSE2 *v2 = static_cast<g2o::VertexSE2 *>(_vertices[1]);

		double p1[3], p2[3];
		v1->getEstimateData(p1);
		v2->getEstimateData(p2);

		double p[3];
		for (int i = 0; i < 3; ++i) {
			p[i] = .5 * (p1[i] + p1[i]);
		}

		_error(0, 0) = 0.0;

//		int VERSION_ID = 0;//SIMPLEST VERSION
		int VERSION_ID = 1;// differen



		for (int i = 0; i < dis_vec_.size(); ++i) {
			double dis = std::sqrt(pow(p[0] - beacon_set_vec_[i].x(), 2.0)
			                       + pow(p[1] - beacon_set_vec_[i].y(), 2.0));

			if (VERSION_ID == 0) {
				if (abs(dis - dis_vec_[i]) > 3.0) {
					_error(0, 0) += 0.0;
				} else {

					_error(0, 0) += abs(dis - dis_vec_[i]);
				}
			}

			if (VERSION_ID == 1) {
				if ((dis_vec_[i] - dis) > 1.0) {
					_error(0, 0) += 0.0;
				} else {

					_error(0, 0) += abs(dis - dis_vec_[i]);

				}
			}




//			printf("range error:%f\n",dis-dis_vec_[i]);
		}
	}

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


	bool setRealMeasurements(std::vector<double> range_vec, std::vector<Eigen::Vector2d> beacon_vec) {
		dis_vec_ = range_vec;
		beacon_set_vec_ = beacon_vec;
		return true;
	}


	/**
	 * Try achieve this function manually.
	 */
//    void linearizeOplus();

/**
 * ...
 * @return
 */
	virtual int measurementDimension() const {
		return 1;
	}

	virtual bool setMeasurementFromState() {

		return true;
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


	/**
	 * initial matrix by value, actually set to zero.
	 * @param from
	 * @param to
	 */
	virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
	                             g2o::OptimizableGraph::Vertex *to) {

	}


	std::vector<Eigen::Vector2d> beacon_set_vec_;
	std::vector<double> dis_vec_;
};


#endif //COMPLEXITYPOSITIONING_DISTANCEEDGESE2POINT2_H
