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

import re

if __name__ == '__main__':
	dir_name = '/home/steve/Data/BJUwbINS/'

	'''
	Imu process
	'''
	imu_data = np.loadtxt(dir_name + 'imu.obs')

	np.savetxt(dir_name + 'imu.data', imu_data, delimiter=',')

	plt.figure()
	plt.subplot(211)
	plt.title('acc')
	plt.plot(imu_data[:, 1:4])
	plt.grid()
	plt.subplot(212)
	plt.title('gyr')
	plt.plot(imu_data[:, 4:7])
	plt.grid()

	'''
	UWB process
	
	'''

	uwb_src_file = open(dir_name + 'uwb.obs')
	uwb_lines = uwb_src_file.readlines()

	uwb_data = np.zeros([len(uwb_lines), 6])
	uwb_data = uwb_data - 10.0

	finder = re.compile('[.0-9]{1,}')

	for i in range(len(uwb_lines)):
		# print(uwb_lines[i].split(' '))
		# print(finder.findall(uwb_lines[i]))
		line_list = finder.findall(uwb_lines[i])
		uwb_data[i, 0] = float(line_list[0])

		for j in range(2, len(line_list), 2):
			uwb_data[i, int(line_list[j])] = float(line_list[j + 1])

	np.savetxt(dir_name + 'uwb_data.csv', uwb_data, delimiter=',')
	plt.figure()
	plt.title('uwb')
	for i in range(1, uwb_data.shape[1]):
		plt.plot(uwb_data[:, 0], uwb_data[:, i], '-+', label=str(i))
	plt.grid()
	plt.legend()

	plt.show()
