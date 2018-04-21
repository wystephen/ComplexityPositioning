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
from mpl_toolkits.mplot3d import Axes3D

import re

if __name__ == '__main__':
	dir_name = '/home/steve/Data/BJUwbINS/'

	'''
	Imu process
	'''
	imu_data = np.loadtxt(dir_name + 'imu.obs')

	new_imu_data = np.zeros([imu_data.shape[0],imu_data.shape[1]+3])
	new_imu_data[:,:imu_data.shape[1]] = imu_data

	np.savetxt(dir_name + 'imu.data', new_imu_data, delimiter=',')

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

	'''
	Uwb Location
	'''

	from Scripts.Trilateration import Trilateration
	from scipy.spatial.distance import *

	beacon_data = np.loadtxt(dir_name+'beaconset_no_mac.csv',delimiter=',')


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
	res_error = tri_positioning.res_error
	src_pose = tri_positioning.location_all((0, 0, 0), uwb_data[:, 1:])
	src_res_error = tri_positioning.res_error

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	# ax.title('position')
	# ax.title
	ax.plot(pose[:, 0], pose[:, 1], pose[:, 2], '*', label='processed pose')
	ax.plot(src_pose[:, 0], src_pose[:, 1], src_pose[:, 2], '+', label='source pose')
	# plt.plot(beacon_data[:, 0], beacon_data[:, 1], '*', label='beacons')
	for i in range(beacon_data.shape[0]):
		if np.max(processed_uwb_data[:, i]) > 0:
			# ax.plot(beacon_data[i, 0], beacon_data[i, 1], label='beacon:' + str(i))
			ax.text(beacon_data[i, 0], beacon_data[i, 1], beacon_data[i, 2], s=str(i))

	ax.legend()
	ax.grid()


	plt.figure()
	plt.title('trace')
	plt.plot(src_pose[:,0],src_pose[:,1],'*',label='src pose')
	plt.plot(pose[:,0],pose[:,1],'*',label='pose')
	plt.legend()
	plt.grid()




	plt.figure()
	plt.title('res error and z-axis value')
	plt.plot(res_error, label='res error')
	plt.plot(pose[:, 2], label='z')
	plt.plot(src_res_error, label='src res error')
	plt.plot(src_pose[:, 2], label='src z')
	plt.grid()
	plt.legend()


	dis_mat = squareform(pdist(uwb_data[:,1:]))
	plt.figure()
	plt.imshow(dis_mat)
	plt.colorbar()
	plt.grid()


	checked_mat = np.zeros_like(dis_mat)
	checked_mat[np.where(dis_mat<1.0)] = 1.0
	plt.figure()
	plt.imshow(checked_mat)
	plt.colorbar()





	plt.show()
