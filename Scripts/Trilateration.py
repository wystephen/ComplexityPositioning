# Created by steve on 18-3-6 下午2:47
'''
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
'''

import numpy as np
import scipy as sp
from scipy.optimize import minimize


class Trilateration:
    def __init__(self, beacon_set):
        self.method = 'Optimization'
        self.beacon_set = beacon_set

    def location_all(self, init_x, uwb_data):
        pose = np.zeros([uwb_data.shape[0], 3])
        for i in range(pose.shape[0]):
            if len(np.where(uwb_data[i, :] > 0.0)[0]) > 2:
                pose[i,:] = self.location((0, 0, 0),
                              uwb_data[i, :])
        return pose


    def location(self, init_x, uwb_measurements):
        # print(init_x)
        self.uwb_m = uwb_measurements

        res = minimize(self.error_function,
                       init_x, method='BFGS')
        # print(res.x)
        # print(res.fun)
        return res.x

    def error_function(self, pose):
        error = 0.0
        dis_all = np.linalg.norm(self.beacon_set - pose, axis=1)
        for i in range(self.beacon_set.shape[0]):
            if self.uwb_m[i] > 0.0:
                error += (dis_all[i] - self.uwb_m[i]) ** 2.0
        return error
