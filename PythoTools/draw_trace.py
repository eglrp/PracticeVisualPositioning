import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	# pose = np.loadtxt('/home/steve/temp/final_pose.csv', delimiter=',')
	pose = np.loadtxt('/home/steve/temp/pose.csv', delimiter=',')

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.plot(pose[:, 0], pose[:, 1], pose[:, 2], '+')


	plt.figure()
	for i in range(3):
		plt.plot(pose[:,i],label=str(i))
	plt.legend()
	plt.grid()

	plt.figure()
	plt.title('2d trace')

	plt.plot(pose[:,0],pose[:,1],'+')
	plt.grid()

	plt.show()
