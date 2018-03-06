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

if __name__ == '__main__':
    dir_name = '/home/steve/Data/FusingLocationData/0017/'

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

    plt.show()
