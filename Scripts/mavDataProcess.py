# Created by steve on 18-3-8 下午5:00
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
import matplotlib.pyplot as plt

if __name__ == '__main__':
    src_data = np.loadtxt('/home/steve/Data/XsensData/data.csv', delimiter=',')
    target_data = np.zeros_like(src_data)
    target_data[:, 0] = src_data[:, 0] / 1e9

    target_data[:, 4] = src_data[:, 3]
    target_data[:, 5] = src_data[:, 1]
    target_data[:, 6] = src_data[:, 2]

    target_data[:, 1] = src_data[:, 6]
    target_data[:, 2] = src_data[:, 4]
    target_data[:, 3] = src_data[:, 5]

    np.savetxt('/home/steve/Data/XsensData/mav_data.csv', target_data, delimiter=',')

    print((target_data[1:, 0] - target_data[:-1, 0]).mean())

    plt.figure()
    for i in range(3):
        plt.plot(target_data[:, i + 1], label=str(i))
    plt.legend()
    plt.grid()

    plt.figure()
    for i in range(3):
        plt.plot(target_data[:, i + 4], label=str(i))

    plt.legend()
    plt.grid()
    plt.show()
