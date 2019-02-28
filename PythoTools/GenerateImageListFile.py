import os

if __name__ == '__main__':
	dir_name = '/home/steve/SourceData/MYNTEYEData/'

	# sub_dir = 'aruco009/'
	sub_dir = 'aruco-hard1/'

	list_file_name = sub_dir[:-1] + '.list'

	list_file = open(dir_name + list_file_name, 'w')

	file_list = os.listdir(dir_name + sub_dir)
	file_list.sort()
	for file in file_list:
		if '.bmp' in file:
			list_file.write(dir_name + sub_dir + file + '\n')
