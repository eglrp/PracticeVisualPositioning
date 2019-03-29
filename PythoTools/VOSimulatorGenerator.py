import numpy as np
import scipy as sp
from mayavi import mlab

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ImuTools import *

from array import array


def generate_trace():
	'''
	Retern 8 trace .
	:return:
	'''
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

	return pos_array, qua_array, angle_array


def generate_feature(x_min,
                     x_max,
                     y_min,
                     y_max,
                     z_min,
                     z_max,
                     x_step=10.0,
                     y_step=10.0,
                     z_step=1.5):
	# x_max = np.max(pos_array[:, 0])
	# x_min = np.min(pos_array[:, 0])
	#
	# y_max = np.max(pos_array[:, 1])
	# y_min = np.min(pos_array[:, 1])
	#
	# z_max = np.max(pos_array[:, 2])
	# z_min = np.min(pos_array[:, 2])

	kp_buf = array('d')

	for xp in np.arange(x_min - 30.0, x_max + 30.0, x_step):
		for yp in np.arange(y_min - 30.0, y_max + 30.0, y_step):
			for zp in np.arange(z_min - 1.0, z_max + 1.0, z_step):
				no = np.random.normal(0.0, 2.0, 3)
				kp_buf.append(xp + no[0] * 3)
				kp_buf.append(yp + no[1] * 3)
				kp_buf.append(zp + no[2])

	kp_array = np.frombuffer(kp_buf, dtype=np.float).reshape([-1, 3])
	return kp_array


def project_to_frame(t,
                     q,
                     kp,
                     cam_size,
                     fx,
                     fy,
                     cx,
                     cy,
                     k1=0.0,
                     k2=0.0,
                     p1=0.0,
                     p2=0.0):
	P = q2dcm(q) @ kp + t
	p = P / P[2]

	def r(p, k1, k2):
		return 1.0 + k1 * (np.linalg.norm(p) ** 2.0) + k1 * (np.linalg.norm(p) ** 4.0)

	xp = fx * r(p, k1, k2) + cx
	yp = fy * r(p, k1, k2) + cy

	if xp > 0 and xp < cam_size[0] and yp > 0 and yp < cam_size[1]:
		return xp, yp
	else:
		return -10.0, -10.0


if __name__ == '__main__':

	pos_array, qua_array, angle_array = generate_trace()

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

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], '-+')
	ax.grid()

	kp_array = generate_feature(np.min(pos_array[:, 0]),
	                            np.max(pos_array[:, 0]),
	                            np.min(pos_array[:, 1]),
	                            np.max(pos_array[:, 1]),
	                            np.min(pos_array[:, 2]),
	                            np.max(pos_array[:, 2]),
	                            20.0, 20.0, 2.0)

	plt.plot(kp_array[:, 0], kp_array[:, 1], kp_array[:, 2], 'Dr')

	print('feature number:', kp_array.shape[0])





	plt.show()
