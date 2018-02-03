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
// Created by steve on 18-2-3.
//

#ifndef COMPLEXITYPOSITIONING_UWBTOOLS_H
#define COMPLEXITYPOSITIONING_UWBTOOLS_H

#include <Eigen/Dense>
namespace BSE{
 class UwbTools {
 public:
     UwbTools(Eigen::MatrixXd uwb_data,Eigen::MatrixXd beacon_set_data):
             uwb_data_(uwb_data),beacon_set_(beacon_set_data){


     }


     Eigen::MatrixXd uwb_data_;
     Eigen::MatrixXd beacon_set_;

};
}



#endif //COMPLEXITYPOSITIONING_UWBTOOLS_H
