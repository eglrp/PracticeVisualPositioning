import numpy as np
import scipy as sp

from matplotlib import pyplot as plt

from array import array
import os

if __name__ == '__main__':
	out_file = open('/home/steve/temp/feature_track_server_out.txt', 'r')

	track_cnt_buf = array('i')
	blur_score_buf = array('f')

	frame_id = 0

	for line_str in out_file:
		# print(line_str)
		if 'track_cnt:' in line_str:
			frame_id += 1
			valid_str = line_str[11:-2]
			# print('valid str:',valid_str)
			# print('valid num len:', len(valid_str.split(',')))
			cur_cnt = int(valid_str.split(',')[0])
			cur_cnt_num = 1
			num_list = valid_str.split(',')
			for i in range(1, len(num_list)):
				# print(i, ':', num_list[i])
				if int(num_list[i]) is cur_cnt:
					cur_cnt_num += 1

				if (not (int(num_list[i]) == cur_cnt)) or \
						(i == len(num_list) - 1):
					track_cnt_buf.append(frame_id)
					# if cur_cnt > 80:
					# 	cur_cnt = 80
					track_cnt_buf.append(cur_cnt)
					track_cnt_buf.append(cur_cnt_num)
					cur_cnt = int(num_list[i])
					cur_cnt_num = 1
		if 'blur_score' in line_str:
			# print(line_str[12:-2])
			blur_score_buf.append(float(line_str[12:-2]))


	t = np.frombuffer(track_cnt_buf, dtype=np.int32).reshape([-1, 3])
	print('index:', np.max(t[:,0]))
	print('cnt:', np.max(t[:,1]))
	dis_img = np.zeros([np.max(t[:, 0]), np.max(t[:,1])])
	for i in range(t.shape[0]):
		dis_img[t[i,0]-1,t[i,1]-1] = t[i,2]

	blur_score = np.frombuffer(blur_score_buf,dtype = np.float32).reshape([-1,1])

	plt.figure()
	plt.imshow(dis_img.transpose(),cmap='ocean_r')
	plt.colorbar()
	plt.tight_layout()

	plt.figure()

	for i in range(dis_img.shape[1]):
		plt.plot(dis_img[:,i],label=str(i))
	plt.legend()
	plt.grid()

	plt.figure()
	plt.subplot(211)
	plt.title('lossed feature')
	plt.plot(dis_img[:,1])
	plt.grid()

	plt.subplot(212)
	plt.title('blur score')
	plt.plot(blur_score[:,0])
	plt.grid()


	plt.show()
