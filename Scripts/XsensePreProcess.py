import matplotlib.pyplot as plt

import scipy as sp
import numpy as np

from numba import jit


@jit
def lowpass_filter(data, rate):
	y = np.zeros_like(data)
	y[0, :] = data[0, :]
	for i in range(1, data.shape[0]):
		y[i, :] = rate * data[i, :] + (1.0 - rate) * y[i - 1, :]
	return y


if __name__ == '__main__':
	dir_name = "/home/steve/Data/XsensUwb/MTI700/0004/"

	imu_data = np.loadtxt(dir_name + 'imu.data', delimiter=',')

	acc = imu_data[:, 1:4]
	gyr = imu_data[:, 4:7]

	acc_rate = 0.1
	gyr_rate = 0.1
	plt.figure()
	plt.title('src acc')
	plt.plot(acc, label='acc')
	plt.plot(lowpass_filter(acc, acc_rate), label='filter')
	plt.legend()
	plt.grid()

	plt.figure()
	plt.title('src gyr')
	plt.plot(gyr, label='gyr')
	plt.plot(lowpass_filter(gyr, gyr_rate), label='filter')
	plt.legend()
	plt.grid()

	plt.show()
