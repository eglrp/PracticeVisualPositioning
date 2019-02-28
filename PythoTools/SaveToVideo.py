import cv2
import numpy as np

fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
output_movie = cv2.VideoWriter('/home/steve/temp/testnights.mp4', fourcc, 25, (752, 480), True)

list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco008.list"

name_list = open(list_file_name).readlines()

for name in name_list:
	print(name)
	img = cv2.imread(name[:-1])
	# cv2.imshow("img", img)
	# cv2.waitKey(10)
	print(img.shape)

	left_img = img[:, :int(img.shape[1] / 2), :] * 1.0
	# print(left_img.shape)
	# cv2.imshow('left',left_img)
	# cv2.waitKey(10)
	# cv2.cvtColor(left_img,left_img,cv2.CV_8UC3)
	print('left image shape', left_img.shape)

	print('is opened:', output_movie.isOpened())
	output_movie.write(np.uint8(left_img))
