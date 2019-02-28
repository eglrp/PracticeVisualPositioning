import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	data = np.loadtxt('/home/steve/test_t.csv', delimiter=',')

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(data[:, 0], data[:, 1], data[:, 2], '-+')
	ax.grid()


	plt.figure()
	plt.plot(data[:, 0], data[:, 1], '-+')
	plt.show()
