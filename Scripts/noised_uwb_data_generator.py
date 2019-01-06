import numpy as np
import scipy as sp
from matplotlib import pyplot as plt

if __name__ == '__main__':
	uwb_data = np.loadtxt('/home/steve/Data/PDRUWBRobust/uwb_noise.csv', delimiter=',')

	noise_data = np.loadtxt('/home/steve/Data/PDRUWBRobust/uwb_noise_data.csv', delimiter=',')

	plt.figure()
	for i in range(noise_data.shape[1]):
		for j in range(noise_data.shape[0]):
			if (noise_data[j, i]) < 1.0:
				noise_data[j, i] = 0.0
		plt.plot(noise_data[:, i], label=str(i))
	plt.grid()
	plt.legend()

	plt.figure()
	plt.subplot(211)
	plt.title('source uwb data')
	for i in range(1, uwb_data.shape[1]):
		plt.plot(uwb_data[:, i], label=str(i))
	plt.legend()
	plt.grid()

	new_uwb_data = uwb_data
	for i in range(uwb_data.shape[0]):
		# new_uwb_data[i, 1:] = uwb_data[i, 1:] + noise_data[i, 1:5]
		# new_uwb_data[i, 1:] = uwb_data[i, 1:] + noise_data[i+200, 0:4]
		new_uwb_data[i, 1:] = uwb_data[i, 1:] + noise_data[i, 2:6]
		new_uwb_data[i, 1:4] = uwb_data[i, 1:4] + noise_data[i + 100, 0:3]

	plt.subplot(212)
	plt.title('noised uwb data')
	for i in range(1, uwb_data.shape[1]):
		plt.plot(new_uwb_data[:, i], label=str(i))
	plt.legend()
	plt.grid()

	np.savetxt('/home/steve/Data/PDRUWBRobust/noised_uwb_datapy.csv', new_uwb_data, delimiter=',')
	import os, subprocess

	os.system('/home/steve/Code/ComplexityPositioning/cmake-build-debug/RobustPDRUWBTester')
	plt.show()
