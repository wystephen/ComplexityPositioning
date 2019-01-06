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


//		int VERSION_ID = 0;//SIMPLEST VERSION
//	int VERSION_ID = 1;// RANSAC
	int VERSION_ID = 2;// robust kernel when system state calculated distance smaller than measurement.

	double robust_threshold_ = 1.0;

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

		double min_error = 1e10;
		int min_error_index = -1;

		for (int i = 0; i < dis_vec_.size(); ++i) {
			double dis = std::sqrt(pow(p[0] - beacon_set_vec_[i].x(), 2.0)
			                       + pow(p[1] - beacon_set_vec_[i].y(), 2.0));
			if (abs(dis - dis_vec_[i]) < min_error) {
				min_error = abs(dis - dis_vec_[i]);
				min_error_index = i;
			}

			if (VERSION_ID == 0) {
				if (abs(dis - dis_vec_[i]) > 3.0) {
					_error(0, 0) += 0.0;
				} else {

					_error(0, 0) += (dis - dis_vec_[i]);
				}
			}

			if (VERSION_ID == 1) {
				if ((dis_vec_[i] - dis) > robust_threshold_) {
					_error(0, 0) += 0.0;
					ransac_flag_vec_[i] = false;
				} else {

					_error(0, 0) += (dis - dis_vec_[i]) * (dis - dis_vec_[i]);
					ransac_flag_vec_[i] = true;

				}
			}
			auto huber_func = [](double value, double eta) -> double {
				if (abs(value) < eta) {
					return 0.5 * value * value;
				} else {
					return eta * (abs(eta) - 0.5 * eta);
				}
			};

			auto tukey_func = [](double u, double d) -> double {
				if (abs(u) < d) {
					return d * d / 6.0 * (1.0 - pow(1.0 - (u * u / d / d), 3.0));
				} else {
					return d * d / 6.0;
				}

			};

			//tukey funct.
			if (VERSION_ID == 2) {
				if ((dis_vec_[i] > dis)) {
					_error(0, 0) += tukey_func(dis - dis_vec_[i], 1.0);
				} else {
					_error(0, 0) += (dis - dis_vec_[i]) * (dis - dis_vec_[i]);
//					_error(0,0) += huber_func(dis-dis_vec_[i],3.0);
				}
			}
//			printf("range error:%f\n",dis-dis_vec_[i]);
		}

		//
		if (VERSION_ID == 1) {
			int true_counter = 0;
			for (int i = 0; i < ransac_flag_vec_.size(); ++i) {
				if (ransac_flag_vec_[i]) {
					true_counter++;
				}
			}
			if (true_counter < 1 and min_error_index > 0 and min_error_index < ransac_flag_vec_.size()) {
				ransac_flag_vec_[min_error_index] = true;
				_error(0, 0) = min_error * min_error;

			}
		}
	}

//
	void linearizeOplus() {
		_jacobianOplusXj.setZero();
		_jacobianOplusXi.setZero();// 1x3 matrix.

		g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2 *>(_vertices[0]);
		g2o::VertexSE2 *v2 = static_cast<g2o::VertexSE2 *>(_vertices[1]);

		double p1[3], p2[3];
		v1->getEstimateData(p1);
		v2->getEstimateData(p2);

		double p[3];
		for (int i = 0; i < 3; ++i) {
			p[i] = .5 * (p1[i] + p1[i]);
		}

		for (int i = 0; i < dis_vec_.size(); ++i) {
			double dis = std::sqrt(pow(p[0] - beacon_set_vec_[i].x(), 2.0)
			                       + pow(p[1] - beacon_set_vec_[i].y(), 2.0));


			if (VERSION_ID == 1) {
				//RANSAC VERSION
				if (ransac_flag_vec_[i] == true) {
//					_jacobianOplusXi(0, 0) += (p[0] - beacon_set_vec_[i].x()) / dis;
//					_jacobianOplusXi(0, 1) += (p[1] - beacon_set_vec_[i].y()) / dis;


					_jacobianOplusXi(0, 0) +=
							(2.0 * (dis - dis_vec_[i]) * (p[0] - beacon_set_vec_[i].x()) / dis);
					_jacobianOplusXi(0, 1) +=
							(2.0 * (dis - dis_vec_[i]) * (p[1] - beacon_set_vec_[i].y()) / dis);
				}


			}

			/**
			 * @brief
			 */
			if (VERSION_ID == 2) {
				//tukey jacobian matrix.
				if ((dis_vec_[i] > dis)) {
					//tukey
					if (dis_vec_[i] > dis + 0.5) {
						_jacobianOplusXi(0, 1) += 0.0;
						_jacobianOplusXi(0, 2) += 0.0;
					} else {


						_jacobianOplusXi(0, 0) += (dis - dis_vec_[i])
						                          *
						                          pow(1.0 - (dis - dis_vec_[i]) * (dis - dis_vec_[i]) / 0.5 / 0.5, 2.0)
						                          * (2.0 * (dis - dis_vec_[i]) * (p[0] - beacon_set_vec_[i].x()) / dis);
						_jacobianOplusXi(0, 1) += (dis - dis_vec_[i])
						                          *
						                          pow(1.0 - (dis - dis_vec_[i]) * (dis - dis_vec_[i]) / 0.5 / 0.5, 2.0)
						                          * (2.0 * (dis - dis_vec_[i]) * (p[1] - beacon_set_vec_[i].y()) / dis);

					}

				} else {
					_jacobianOplusXi(0, 0) +=
							(2.0 * (dis - dis_vec_[i]) * (p[0] - beacon_set_vec_[i].x()) / dis);
					_jacobianOplusXi(0, 1) +=
							(2.0 * (dis - dis_vec_[i]) * (p[1] - beacon_set_vec_[i].y()) / dis);


				}
			}


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
		assert(dis_vec_.size() == beacon_set_vec_.size());
		for (int i = 0; i < dis_vec_.size(); ++i) {
			ransac_flag_vec_.push_back(true);
		}
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
	std::vector<bool> ransac_flag_vec_;
};


#endif //COMPLEXITYPOSITIONING_DISTANCEEDGESE2POINT2_H
