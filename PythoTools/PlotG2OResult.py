import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from mayavi import mlab

from array import array

if __name__ == '__main__':

	file_name = '/home/steve/temp/local_pose_result.g2o'

	pos_buf = array('d')
	tag_buf = array('d')

	x_offset_ = 100000000
	c_offset_ = 200000000
	m_offset_ = 300000000

	file_all_lines = open(file_name).readlines()

	for current_line in file_all_lines:
		if 'VERTEX_SE3' in current_line:
			all_units = current_line.split(' ')
			print('all units:', all_units)

			if abs(int(all_units[1]) - c_offset_) < 1e6:
				# x (central points)
				for i in range(3):
					pos_buf.append(float(all_units[i + 2]))

				pos_buf.append(float(all_units[8]))

				for i in range(3):
					pos_buf.append(float(all_units[i + 5]))

				pass

			if abs(int(all_units[1]) - c_offset_) < 1e6:
				# camera index
				pass

			if abs(int(all_units[1]) - m_offset_) < 1e6:
				# marker index
				for i in range(3):
					tag_buf.append(float(all_units[i + 2]))

				tag_buf.append(float(all_units[8]))

				for i in range(3):
					tag_buf.append(float(all_units[i + 5]))
				pass

	pos_array = np.frombuffer(pos_buf, dtype=np.float).reshape([-1, 7])
	tag_array = np.frombuffer(tag_buf, dtype=np.float).reshape([-1, 7])

	from PythoTools.TrajectoryVisualization3D import *

	visualize_trajectory(pos_array, 'pos array', 1, 1)
	visualize_trajectory(tag_array, 'tag', 2, 1)
	mlab.sync_camera(mlab.figure(1),mlab.figure(2))
	# mlab.draw()
	from mpl_toolkits.mplot3d import Axes3D

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], )

	plt.figure()
	plt.title('trace')
	for i in range(3):
		plt.plot(pos_array[:, i], label=str(i))
	plt.legend()
	plt.grid()
	plt.show()
