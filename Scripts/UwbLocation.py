# Created by steve on 18-3-6 下午2:04
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

from Scripts.Trilateration import Trilateration

if __name__ == '__main__':
    dir_name = '/home/steve/Data/FusingLocationData/0013/'

    uwb_data = np.loadtxt(dir_name + 'uwb_result.csv', delimiter=',')
    beacon_data = np.loadtxt(dir_name + 'beaconSet.csv', delimiter=',')

    plt.figure()
    plt.title('source data')

    for i in range(1, uwb_data.shape[1]):
        plt.plot(uwb_data[:, 0],
                 uwb_data[:, i],
                 '*',
                 label=str(i))
    plt.grid()
    plt.legend()

    first_order_div = uwb_data[1:, 1:] - uwb_data[:-1, 1:]
    second_order_div = first_order_div[1:, :] - first_order_div[:-1, :]

    plt.figure()
    plt.title('div')

    for i in range(first_order_div.shape[1]):
        # plt.plot(first_order_div[:, i],
        #          '*',
        #          label='first' + str(i))
        plt.plot(second_order_div[:, i],
                 '+',
                 label='second' + str(i))

    plt.legend()
    plt.grid()

    processed_uwb_data = uwb_data[1:-1, 1:].copy()

    processed_uwb_data[np.where(np.abs(second_order_div) > 0.6)] = -10.0

    plt.figure()
    plt.title('processed')
    for i in range(processed_uwb_data.shape[1]):
        plt.plot(processed_uwb_data[:, i],
                 '*',
                 label='processed' + str(i))
    plt.legend()
    plt.grid()
    # plt.show()

    tri_positioning = Trilateration(beacon_set=beacon_data)
    pose = tri_positioning.location_all((0, 0, 0), processed_uwb_data)
    src_pose = tri_positioning.location_all((0, 0, 0), uwb_data[:, 1:])

    plt.figure()
    plt.title('position')
    plt.plot(pose[:, 0], pose[:, 1], '*', label='processed pose')
    plt.plot(src_pose[:, 0], src_pose[:, 1], '*', label='source pose')
    plt.legend()
    plt.grid()
    plt.show()

    # plt.show()
