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
// Created by steve on 18-3-25.
//

#ifndef COMPLEXITYPOSITIONING_UWBMEASUREMENTFUNCTION_H
#define COMPLEXITYPOSITIONING_UWBMEASUREMENTFUNCTION_H

#include <AWF.h>

class UwbMeasurementFunction : public AWF::SingleFunctionAbstract {
public:
    UwbMeasurementFunction(Eigen::MatrixXd beacon_set) :
            SingleFunctionAbstract(1) {
        beacon_set_ = beacon_set;

    }

    /**
     * There is not other condition except the first three dimension should be pose.
     * @param in
     * @return
     */
    Eigen::MatrixXd compute(Eigen::MatrixXd in) {
        auto t = Eigen::MatrixXd(OutDim, 1);
        t(0, 0) = (in.block(0, 0, 3, 1) - beacon_set_).norm();
        return t;

    }


    Eigen::MatrixXd beacon_set_; // beaconset.

};


#endif //COMPLEXITYPOSITIONING_UWBMEASUREMENTFUNCTION_H
