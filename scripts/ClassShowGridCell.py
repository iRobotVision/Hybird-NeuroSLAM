#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy.ndimage import zoom

class ClassShowGridCell:
    def __init__(self):
        self.boxsize = 15
        self.step = 0.1
        self.matix_size = int(self.boxsize / self.step)
        self.matrix_show_factor = 5  # 将matix放大的倍数（放大后轨迹会更精细）

        self.trajectory = []  # 保存坐标轨迹

        self.fig, self.axes = plt.subplots(nrows=1, ncols=1, figsize=(5.5, 5))
        # 隐藏刻度
        plt.xticks([])
        plt.yticks([])
        # 设置窗口标题
        self.fig.canvas.manager.set_window_title('Position Estmation')


        # 初始化图像
        self.imgs = self.axes.imshow(np.random.random((self.boxsize, self.boxsize)) - self.boxsize/2,
                                     animated=True,  # 指定图像为动画
                                     extent=[-self.boxsize/2, self.boxsize/2, -self.boxsize/2, self.boxsize/2],  # 指定图像的显示范围
                                     interpolation='gaussian', # 指定图像的插值方式
                                     cmap='twilight_shifted'  # 蓝-紫-粉-橙色调
                                     )
        self.imgs.set_data(self.gaussian2D([0, 0], [1.0, 1.0]))
        self.imgs.set_clim(vmin=0, vmax=0.16)


        # 先保存背景，之后每次更新先恢复再绘制。这就避免了绘制整个窗口，提高了性能。
        self.backgrounds = self.fig.canvas.copy_from_bbox(self.axes.bbox)

        # 显示图形
        self.fig.canvas.draw()
        # 恢复背景
        self.fig.canvas.restore_region(self.backgrounds)
        # 绘制图像
        self.axes.draw_artist(self.imgs)
        # 将更新后的图像显示到图形窗口中
        self.fig.canvas.blit(self.axes.bbox)
        # 显示窗口
        self.fig.show()

        plt.ion()

    """
    构建2D高斯分布
    :param mu: 均值
    :param act_level: 方差的倒数
    """
    def gaussian2D(self, mean, act_level):
        mu_x = mean[0]
        mu_y = mean[1]
        act_level_x = act_level[0]
        act_level_y = act_level[1]

        x = np.arange(-self.boxsize, self.boxsize * 2, self.step, float)
        # x = np.arange(0, self.boxsize, self.step, float).
        y = x[:, np.newaxis]
        mat = np.sqrt(act_level_x * act_level_y) / (2.0 * math.pi) * np.exp(
            - (np.power(x - mu_x, 2.0) * act_level_x / 2.0
               + np.power(y - mu_y, 2.0) * act_level_y / 2.0)
        )

        retMat = np.zeros((self.matix_size, self.matix_size))
        for i in np.arange(0, 3, 1):
            for j in np.arange(0, 3, 1):
                retMat += mat[self.matix_size * i: self.matix_size * (i + 1),
                              self.matix_size * j: self.matix_size * (j + 1)]
        return retMat


    """
    在图像矩阵中加入轨迹
    """
    def add_traj(self, matrix):
        matrix = matrix.T
        # 将矩阵放大(不然轨迹会很粗糙)
        zoomed_matrix = zoom(matrix, self.matrix_show_factor)
        for i in range(len(self.trajectory)):
            x = self.trajectory[i][0]
            y = self.trajectory[i][1]
            if 0 <= x < self.boxsize and 0 <= y < self.boxsize:
                zoomed_matrix[int(x /self.step * self.matrix_show_factor), int(y/self.step * self.matrix_show_factor)] = 1
        return zoomed_matrix

    def wrap(self, x):
        x = x+self.boxsize/2
        if x < 0:
            x = self.boxsize + x
        if x >= self.boxsize:
            x = x - self.boxsize
        return x

    def showGridCell(self, curr_coordinate):
        if curr_coordinate[3] == 0:
            curr_coordinate[3] = 0.0001
        if curr_coordinate[5] == 0:
            curr_coordinate[5] = 0.0001

        # mean_x = curr_coordinate[2]
        mean_x = self.wrap(curr_coordinate[2])  # 使初始化在图像中心
        act_level_x = curr_coordinate[3]  # act_level = 1/sigma^2
        # mean_y = curr_coordinate[4]
        mean_y = self.wrap(-curr_coordinate[4])
        act_level_y = curr_coordinate[5]
        if act_level_x > 0.8 and act_level_y > 0.8:
            # 添加当前坐标到轨迹列表（回避方差过大的情况）
            # self.trajectory.append([mean_x, mean_y])
            self.trajectory.append([mean_y, mean_x])

        # print('mean_x:', mean_x, 'mean_y:', mean_y)

        # matrix_curr = self.gaussian2D([mean_x, mean_y], [act_level_x, act_level_y])
        matrix_curr = self.gaussian2D([mean_y, mean_x], [act_level_y, act_level_x])
        # matrix_traj = matrix_curr[::-1]
        matrix_traj = self.add_traj(matrix_curr)


        self.fig.canvas.restore_region(self.backgrounds)
        self.imgs.set_data(matrix_traj)
        self.axes.draw_artist(self.imgs)
        self.fig.canvas.blit(self.axes.bbox)

        self.fig.canvas.flush_events()


if __name__ == "__main__":
    sh2D = ClassShowGridCell()

    curr_coordinate = [0] * 6
    curr_coordinate[2] = -3.0
    curr_coordinate[3] = 1.0
    curr_coordinate[4] = -5.0
    curr_coordinate[5] = 1.0

    tstart = time.time()
    sh2D.showGridCell(curr_coordinate)
    print('FPS:', 2000 / (time.time() - tstart))
    plt.show()
