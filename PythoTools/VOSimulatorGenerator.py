import numpy as np
import scipy as sp
from mayavi import mlab

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ImuTools import *

from array import array

from numba import jit, njit, prange


@jit
def get_zoffset(z_offset, z_sign):
	z_offset = z_offset + (z_sign * 1.0 / 180.0)
	if z_offset > 2.0:
		z_sign = -1.0  # z_sign * -1.0
	if z_offset < -2.0:
		z_sign = 1.0  # z_sign * -1.0
	return z_offset, z_sign


@jit
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

	for i in range(1800):
		angle = dcm2euler(q2dcm(init_q))
		angle_buf.append(angle[0])
		angle_buf.append(angle[1])
		angle_buf.append(angle[2])

		init_q = quaternion_right_update(init_q,
		                                 np.asarray((0, 0.0, np.pi)), 1.0 / 900.0)
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
		                                 np.asarray((0, 0.0, np.pi)), -1.0 / 900.0)
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


@jit(cache=True)
def generate_feature(x_min,
                     x_max,
                     y_min,
                     y_max,
                     z_min,
                     z_max,
                     x_step=10.0,
                     y_step=10.0,
                     z_step=1.5):
	kp_buf = array('d')

	for xp in np.arange(x_min - 30.0, x_max + 30.0, x_step):
		for yp in np.arange(y_min - 30.0, y_max + 30.0, y_step):
			for zp in np.arange(z_min - 1.0, z_max + 1.0, z_step):
				no = np.random.normal(0.0, 5.0, 3)
				kp_buf.append(xp + no[0] * 3)
				kp_buf.append(yp + no[1] * 3)
				kp_buf.append(zp + no[2])

	kp_array = np.frombuffer(kp_buf, dtype=np.float).reshape([-1, 3])
	return kp_array


@jit
def r(p, k1, k2):
	# return 1.0 + k1 * (np.linalg.norm(p) ** 2.0) + k2 * (np.linalg.norm(p) ** 4.0)
	return 1.0


@jit(cache=True)
def project_to_frame(t,
                     q,
                     r_t,
                     r_q,
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
	# P = np.linalg.inv(q2dcm(q)) @ (kp  t)
	P = ((q2dcm(q)) @ (kp - t))
	P = (q2dcm(r_q) @ (P - r_t))
	p = -1.0 * P / P[2]

	xp = fx * r(p, k1, k2) * p[0] + cx
	yp = fy * r(p, k1, k2) * p[1] + cy

	# return xp,yp
	if xp > 0 and xp < cam_size[0] and yp > 0 and yp < cam_size[1]:
		return xp, yp
	else:
		return -10.0, -10.0


@jit(parallel=True)
def project_all_frame(pos_array, qua_array, kp_array):
	frame_array = np.zeros([pos_array.shape[0], kp_array.shape[0], 2])
	r_frame_array = np.zeros([pos_array.shape[0], kp_array.shape[0], 2])
	for i in prange(frame_array.shape[0]):
		for j in range(frame_array.shape[1]):
			xp, yp = project_to_frame(pos_array[i, :], qua_array[i, :],
			                          np.asarray((0.0, 0.0, 0.0)), np.asarray((1.0, 0.0, 0.0, 0.0)),
			                          kp_array[j, :],
			                          np.asarray((1280, 720)),
			                          500.0, 500.0, 640.0, 360.0)
			rxp, ryp = project_to_frame(pos_array[i, :], qua_array[i, :],
			                            np.asarray((0.5, 0.0, 0.0)), np.asarray((1.0, 0.0, 0.0, 0.0)),
			                            kp_array[j, :],
			                            np.asarray((1280, 720)),
			                            500.0, 500.0, 640.0, 360.0)
			frame_array[i, j, 0] = xp
			frame_array[i, j, 1] = yp
			r_frame_array[i, j, 0] = rxp
			r_frame_array[i, j, 1] = ryp
	return frame_array, r_frame_array


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

	kp_array = generate_feature(np.min(pos_array[:, 0]) - 50.0,
	                            np.max(pos_array[:, 0]) + 50.0,
	                            np.min(pos_array[:, 1]) - 50.0,
	                            np.max(pos_array[:, 1]) + 50.0,
	                            np.min(pos_array[:, 2]) - 50.0,
	                            np.max(pos_array[:, 2]) + 50.0,
	                            25.0, 25.0, 20.0)

	plt.plot(kp_array[:, 0], kp_array[:, 1], kp_array[:, 2], '+y')

	# frame_array, r_frame_array = project_all_frame(pos_array, qua_array, kp_array)

	print('feature number:', kp_array.shape[0])

	# plt.figure()
	# plt.title('frame'+str(i))
	# plt.plot(frame_array[i,:,0],frame_array[i,:,1],'y+',label='left')
	# plt.plot(r_frame_array[i,:,0],r_frame_array[i,:,1],'r+',label='right')
	# plt.axis('equal')
	# plt.grid()
	# plt.legend()
	# plt.show()

	# np.savetxt('/home/steve/temp/sim_frame_kpts.csv',
	#            frame_array.reshape([pos_array.shape[0], kp_array.shape[0] * 2]),
	#            delimiter=',')
	# np.savetxt('/home/steve/temp/sim_rframe_kpts.csv',
	#            r_frame_array.reshape([pos_array.shape[0], kp_array.shape[0] * 2]),
	#            delimiter=',')
	np.savetxt('/home/steve/temp/sim_qua.csv',
	           qua_array,
	           delimiter=',')
	np.savetxt('/home/steve/temp/sim_pos.csv',
	           pos_array,
	           delimiter=',')
	np.savetxt('/home/steve/temp/kp_points.csv',
	           kp_array, delimiter=',')

	plt.show()
