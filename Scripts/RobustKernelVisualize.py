import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

if __name__ == '__main__':
	x = np.linspace(-1.5, 1.5, 600)


	def diff(x, func):
		return (func[1:] - func[:-1]) / (x[1] - x[0])


	def tukey(all_x, eta=1.0):
		# return eta * eta / 5.0 * (1.0 - (1.0 - x * x / eta / eta) ** 3.0)
		out = np.zeros_like(all_x)
		for i in range(all_x.shape[0]):
			x = all_x[i]
			if abs(x) < eta:
				out[i] = eta * eta / 6.0 * (1.0 - (1.0 - x * x / eta / eta) ** 3.0)
			else:
				out[i] =  eta * eta / 6.0
		return out



	def huber(all_x, eta=1.0):
		out = np.zeros_like(all_x)
		for i in range(all_x.shape[0]):
			x = all_x[i]
			if abs(x) < eta:
				out[i] = 0.5 * x * x
			else:
				out[i] = eta * (abs(x) - 0.5 * eta)

		return out


	plt.figure()
	plt.plot(x, tukey(x, 1.0), label='tukey')
	plt.plot(x[1:], diff(x, tukey(x)), label='d tukey')
	plt.plot(x, huber(x), label='huber')
	plt.plot(x[1:], diff(x, huber(x)), label='d huber')

	plt.legend()
	plt.grid()

	plt.show()
