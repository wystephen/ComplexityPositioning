# Created by steve on 18-3-17 下午7:18
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

import matplotlib.pyplot as plt



if __name__ == '__main__':
    dir_name  = "/home/steve/Data/XsensUwb/MTI700/0003/"

    imu_data = np.loadtxt(dir_name+'imu.data',delimiter=',')
    uwb_data =np.loadtxt(dir_name+'uwb_data.csv',delimiter=',')
    beacon_data = np.loadtxt(dir_name+'beaconset_no_mac.csv',delimiter=',')

    plt.figure()

    plt.plot(imu_data[:,0],label = 'imu time')
    plt.plot(uwb_data[:,0]-uwb_data[0,0]+imu_data[0,0], label=' uwb time')

    plt.grid()
    plt.legend()

    plt.show()
