# Created by steve on 18-2-7 下午4:17
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
from matplotlib.pylab import plt


class DataLoader:
    def __init__(self,
                 dir_name='/home/steve/Data/FusingLocationData/0013/'):
        if not dir_name[-1] is '/':
            dir_name = dir_name + '/'

        imu_left = np.loadtxt(dir_name+'LEFT_FOOT.data',delimiter=',')
        imu_right = np.loadtxt(dir_name+'RIGHT_FOOT.data',delimiter=',')
        imu_head = np.loadtxt(dir_name+'HEAD.data',delimiter=',')

        uwb_head = np.loadtxt(dir_name + 'uwb_result.csv',delimiter=',')
        beacon_set = np.loadtxt(dir_name+'beaconSet.csv',delimiter=',')

        print('average time interval of left:',
              float(imu_left[-1,1]-imu_left[0,1])/float(imu_left.shape[0]))



if __name__ == '__main__':
    dl = DataLoader()
