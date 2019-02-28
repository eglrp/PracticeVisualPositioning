# -*- coding:utf-8 -*-
# carete by steve at  2018 / 12 / 09　8:36 PM
'''
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
'''
import numpy as np

from ImuTools import *


def visualize_trajectory(gt, title_name='defualt',id=0,step=10):#, file_name='3d_trajectory.csv', dir_name='/home/steve/MatlabWs/VisualizationToolbox/'):
    trajectory = np.zeros([gt.shape[0], 12])
    trajectory[:, 0:3] = gt[:, 0:3] * 1.0
    x_vec = np.asarray((1.0, 0.0, 0.0))
    y_vec = np.asarray((0.0, 1.0, 0.0))
    z_vec = np.asarray((0.0, 0.0, 1.0))
    for i in range(0, trajectory.shape[0], step):
        # trajectory[:,3:6] = np.linalg.inv(q2dcm(dg.gt[i,4:8])).dot(x_vec)
        # trajectory[:,3:6] = (q2dcm(dg.gt[i,4:8])).dot(x_vec)
        R = np.linalg.inv(q2dcm(gt[i, 3:7]))
        # trajectory[i, 0:3] = gt[i, 1:4]
        trajectory[i, 3:6] = R.dot(x_vec)
        trajectory[i, 6:9] = R.dot(y_vec)
        trajectory[i, 9:12] = R.dot(z_vec)
    # np.savetxt(dir_name + file_name, trajectory,delimiter=',')

    from mayavi import mlab

    # ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],label='trajector')
    if id > 0:

        fig = mlab.figure(id)
    else:
        fig = mlab.figure()
    # fig.title(title_name)
    # fig.
    mlab.plot3d(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], tube_radius=0.01,color=(1.0,1.0,1.0))
    t_name = title_name
    mlab.title(t_name)
    for k in range(3):
        c = np.zeros(3)
        c[k] = 1.0
        ct = (c[0], c[1], c[2])

        quiver = mlab.quiver3d(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                               trajectory[:, 3 + k * 3], trajectory[:, 4 + k * 3], trajectory[:, 5 + k * 3],
                               color=ct)
        # quiver.glyph.mask_input_points = True
        # quiver.glyph.mask_points.on_ratio=20
    # if id==2:
    #     mlab.sync_camera(mlab.figure(1),)
    mlab.axes()
    mlab.draw(fig)



    # ax.grid()
    # ax.legend()
