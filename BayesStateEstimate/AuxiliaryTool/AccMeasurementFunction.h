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
// Created by steve on 18-3-10.
//

#ifndef COMPLEXITYPOSITIONING_ACCMEASUREMENTFUNCTION_H
#define COMPLEXITYPOSITIONING_ACCMEASUREMENTFUNCTION_H
#include <Eigen/Dense>
#include "AWF.h"

class AccMeasurementFunction:public AWF::FunctionAbstract {
public:
    AccMeasurementFunction(int out_dim, int in_number) : FunctionAbstract(out_dim, in_number) {

    }


};


#endif //COMPLEXITYPOSITIONING_ACCMEASUREMENTFUNCTION_H
