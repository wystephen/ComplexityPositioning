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

	DistanceEdge2D(){
		_information.setIdentity();
	}

	virtual bool read(std::istream &is);

	virtual bool write(std::ostream &os) const;

	virtual void computeError();

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


	bool setRealMeasurements(std::vector<double> range_vec, std::vector<Eigen::Vector2d> beacon_vec){
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

	virtual bool setMeasurementFromState(){
		
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
	                             g2o::OptimizableGraph::Vertex *to){
		
	}


	double sigma_ = 1.0;

	bool setSigma(double sigma) {
		if (sigma > 0) {
			sigma_ = sigma;
			return true;
		} else {
			return false;
		}

	}

	/**
	 *
	 * @return sigma in error function.
	 */
	double getSigma() {
		return sigma_;
	}

	inline double logNormalPdf(double x, double miu, double sigma) {
		double para1((x - miu) * (x - miu) / 2 / sigma / sigma);
		double para2(1 / std::sqrt(2 * sigma * sigma * M_PI));

		return std::log(para2 + 1e-10) / (para1 + 1e-10);
	}

	inline double NormalPdf(double x,
	                        double miu,
	                        double sigma) {
//    std::cout << "dis :" << x << " range:" << miu << std::endl;
		double para1((x - miu) * (x - miu) / 2 / sigma / sigma);
		double para2(1 / std::sqrt(2 * sigma * sigma * M_PI));
//
//        if(!(!std::isinf(para1)&&!std::isnan(para1)&&!std::isinf(para2)&&!std::isnan(para2)))
//        {
//            std::cout << para1<< " " << para2 << std::endl;
//            std::cout << "x:" << x << "miu:" << miu << "sigma:" << sigma << std::endl;
//        }
		assert(!std::isinf(para1) && !std::isnan(para1) && !std::isinf(para2) && !std::isnan(para2));
		return para2 * std::exp(-para1);
	}


	std::vector<Eigen::Vector2d> beacon_set_vec_;
	std::vector<double> dis_vec_;
};


#endif //COMPLEXITYPOSITIONING_DISTANCEEDGESE2POINT2_H
