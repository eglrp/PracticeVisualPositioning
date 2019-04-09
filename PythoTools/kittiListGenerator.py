import numpy as np
import scipy as sp
from matplotlib import pyplot as plt

import os

if __name__ == '__main__':
	dir_name = '/home/steve/SourceData/Kittidataset/sequences/02/'

	sub_dir_name = 'image_0'

	print(os.listdir(dir_name + sub_dir_name))

	file_name_list = os.listdir(dir_name + sub_dir_name)

	file_name_list = sorted(file_name_list)

	list_file = open(dir_name + sub_dir_name + '.list', 'w')

	for file_name in file_name_list:
		if 'png' in file_name:
			# print(file_name,'\n')
			# list_file.writelines(file_name)
			list_file.write(file_name + '\n')

	list_file.close()

	pose_file = dir_name.replace('sequences', 'poses')[:-1] + '.txt'
	print('pose file:', pose_file)

	pose_data = np.loadtxt(pose_file, delimiter=' ')
	print('min  max std')

	for i in range(pose_data.shape[1]):
		print(i, np.min(pose_data[:, i]),
		      np.max(pose_data[:, i]),
		      np.std(pose_data[:, i]),
		      np.max(pose_data[:, i]) - np.min(pose_data[:, i]))
	#     plt.figure()
	#     plt.title(str(i))
	#     plt.plot(pose_data[:,i])
	#     plt.grid()
	# print(np.min(pose_data,axis=0))
	# print(np.max(pose_data,axis=0))

	from mpl_toolkits.mplot3d import Axes3D

	fig = plt.figure()
	ax = fig.add_subplot(111,projection='3d')

	ax.plot(pose_data[:,3],pose_data[:,7],pose_data[:,11])
	ax.grid()

	calib_file = dir_name+'calib.txt'

	str_linex = open(calib_file).readlines()

	cam1_coeff = np.asarray(str_linex[0].split(' ')[1:]).reshape([3,4])
	print('cam1 parameters:', cam1_coeff)




	plt.show()
