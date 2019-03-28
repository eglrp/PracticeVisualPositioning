import numpy as np
import scipy as sp
from mayavi import mlab

import matplotlib.pyplot as plt

from ImuTools import *

from array import array

if __name__ == '__main__':

	init_q = np.asarray((1.0, 0, 0, 0))
	pos = np.asarray((0, 0, 0))

	angle_buf = array('d')
	qua_buf = array('d')
	pos_buf = array('d')

	z_offset = 0.0
	z_sign = 1.0


	def get_zoffset(z_offset, z_sign):
		z_offset = z_offset + (z_sign * 1.0 / 180.0)
		if z_offset > 2.0:
			z_sign = -1.0  # z_sign * -1.0
		if z_offset < -2.0:
			z_sign = 1.0  # z_sign * -1.0
		return z_offset, z_sign


	for i in range(1800):
		angle = dcm2euler(q2dcm(init_q))
		angle_buf.append(angle[0])
		angle_buf.append(angle[1])
		angle_buf.append(angle[2])

		init_q = quaternion_right_update(init_q,
		                                 np.asarray((0, 0, np.pi)), 1.0 / 900.0)
		pos = pos + (q2dcm(init_q).dot(np.asarray((0.1, 0, 0))))

		z_offset, z_sign = get_zoffset(z_offset, z_sign)
		pos[2] = z_offset

		pos_buf.append(pos[0])
		pos_buf.append(pos[1])
		pos_buf.append(pos[2])

		qua_buf.append(init_q[0])
		qua_buf.append(init_q[1])
		qua_buf.append(init_q[2])
		qua_buf.append(init_q[3])

	for i in range(1800):
		angle = dcm2euler(q2dcm(init_q))
		angle_buf.append(angle[0])
		angle_buf.append(angle[1])
		angle_buf.append(angle[2])

		init_q = quaternion_right_update(init_q,
		                                 np.asarray((0, 0, np.pi)), -1.0 / 900.0)
		pos = pos + (q2dcm(init_q).dot(np.asarray((0.1, 0, 0))))

		z_offset, z_sign = get_zoffset(z_offset, z_sign)
		pos[2] = z_offset

		pos_buf.append(pos[0])
		pos_buf.append(pos[1])
		pos_buf.append(pos[2])

		qua_buf.append(init_q[0])
		qua_buf.append(init_q[1])
		qua_buf.append(init_q[2])
		qua_buf.append(init_q[3])

	angle_array = np.frombuffer(angle_buf, dtype=np.float).reshape([-1, 3])
	pos_array = np.frombuffer(pos_buf, dtype=np.float).reshape([-1, 3])
	qua_array = np.frombuffer(qua_buf, dtype=np.float).reshape([-1, 4])

	plt.figure()
	for i in range(3):
		plt.plot(angle_array[:, i], '-', label=str(i))

	plt.grid()
	plt.legend()

	plt.figure()
	for i in range(3):
		plt.plot(pos_array[:, i], '-', label=str(i))
	plt.legend()
	plt.grid()

	plt.figure()
	plt.plot(pos_array[:, 0], pos_array[:, 1], '-+')
	plt.grid()

	from mpl_toolkits.mplot3d import Axes3D

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], '-+')
	ax.grid()

	x_max = np.max(pos_array[:, 0])
	x_min = np.min(pos_array[:, 0])

	y_max = np.max(pos_array[:, 1])
	y_min = np.min(pos_array[:, 1])

	z_max = np.max(pos_array[:, 2])
	z_min = np.min(pos_array[:, 2])

	kp_buf = array('d')

	for xp in np.arange(x_min - 15.0, x_max + 15.0, 10.0):
		for yp in np.arange(y_min - 15.0, y_max + 15.0, 10.0):
			for zp in np.arange(z_min - 1.0, z_max + 1.0, 1.5):
				no = np.random.normal(0.0, 0.1, 3)
				kp_buf.append(xp + no[0])
				kp_buf.append(yp + no[1])
				kp_buf.append(zp + no[2])

	kp_array = np.frombuffer(kp_buf, dtype=np.float).reshape([-1, 3])

	plt.plot(kp_array[:, 0], kp_array[:, 1], kp_array[:, 2], 'Dr')






	plt.show()
