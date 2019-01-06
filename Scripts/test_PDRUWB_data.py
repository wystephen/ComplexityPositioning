import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

if __name__ == '__main__':
	pdr_data = np.loadtxt('/home/steve/Data/PDRUWBRobust/pdr_result.csv', delimiter=',')

	from array import array
	import math

	trace = np.zeros([pdr_data.shape[0], 3])

	initial_pos = np.asarray((0.0, 0.0, 0.0))

	for i in range(pdr_data.shape[0]):

		trace[i, 2] = initial_pos[2] + pdr_data[i, 3]

		while trace[i, 2] > np.pi:
			trace[i, 2] -= (2.0 * np.pi)

		while trace[i, 2] < -np.pi:
			trace[i, 2] += (2.0 * np.pi)

		dx = pdr_data[i, 1] * math.cos(trace[i, 2])
		dy = pdr_data[i, 1] * math.sin(trace[i, 2])
		# dx = pdr_data[]

		trace[i, 0] = initial_pos[0] + dx
		trace[i, 1] = initial_pos[1] + dy



		initial_pos = trace[i, :]

	plt.figure()
	plt.plot(trace[:, 0], trace[:, 1])
	plt.grid()

	plt.figure()
	plt.plot(trace[:, 2])
	plt.grid()
	plt.show()
