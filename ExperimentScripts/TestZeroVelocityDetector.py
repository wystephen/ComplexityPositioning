# Created by steve on 18-1-30 下午3:40
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
from matplotlib import  pyplot as plt

if __name__ == '__main__':
    imu_data = np.loadtxt('/home/steve/Data/FusingLocationData/0014/LEFT_FOOT.data',
                          delimiter=',')

    imu_data = imu_data[:,1:]

    imu_data[:,1:4] *= 9.81
    imu_data[:,4:7] *= (np.pi /180.0)

    plt.figure()
    plt.plot(imu_data[:,0],
             np.linalg.norm(imu_data[:,1:4],axis=1),
             '*-',label='norm acc')
    plt.plot(imu_data[:,0],
             np.linalg.norm(imu_data[:,4:7],axis=1),
             '*-',label='norm gyr')
    zupt_flag =np.ones_like(imu_data[:,0])
    for i in range(10,imu_data.shape[0]-10):
        if(np.linalg.norm(imu_data[i-4:i+4,4:7],axis=1).mean()<0.16):
            zupt_flag[i] = 1
        else:
            zupt_flag[i] = 0

    plt.plot(imu_data[:,0],
             zupt_flag[:],'*-',label='flag')
    plt.legend()
    plt.grid()



    plt.show()
