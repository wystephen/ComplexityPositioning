//
// Created by steve on 18-4-29.
//

#ifndef COMPLEXITYPOSITIONING_PESUDORANSACDISTANCE_H
#define COMPLEXITYPOSITIONING_PESUDORANSACDISTANCE_H
#include "DistanceEdge.h"
//#include "DistanceEdge.cpp"

class PesudoRansacDistance:public DistanceEdge {
public:
	PesudoRansacDistance(){

	}

	bool ransac_flag_ = false;
	double ransac_threshold_ = 10.0;

	virtual void computeError(){
			g2o::VertexSE3 *from = static_cast<g2o::VertexSE3 *>(_vertices[0]);
			g2o::VertexSE3 *to = static_cast<g2o::VertexSE3 *>(_vertices[1]);

			double p1[10], p2[10];
			from->getEstimateData(p1);
			to->getEstimateData(p2);

			double dis_2 = (p1[0] - p2[0]) * (p1[0] - p2[0]) +
			               (p1[1] - p2[1]) * (p1[1] - p2[1]) +
			               (p1[2] - p2[2]) * (p1[2] - p2[2]);
			double dis = sqrt(dis_2);
			dis = (pow(dis-_measurement,2.0));
//        std::cout << dis_2 << std::endl;
		if(ransac_flag_){
			// ransac model
			if( dis > ransac_threshold_){

				_error(0,0) = 0.0;
			}else{
				_error(0,0)=dis;
			}

		}else{

			_error(0, 0) = dis;
		}
	}

};


#endif //COMPLEXITYPOSITIONING_PESUDORANSACDISTANCE_H
