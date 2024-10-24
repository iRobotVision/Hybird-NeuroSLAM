from matplotlib import pyplot as plt
import numpy as np
import math


class ClassShowHDCell:
    def __init__(self):
        self.fig_, self.ax_ = plt.subplots(subplot_kw={'projection': 'polar'})
        self.fig_.canvas.manager.set_window_title('Head Direction Cell')

        # self.fig_.canvas.draw()
        # self.fig_.patch.set_facecolor('white')


        self.linehl_, = self.ax_.plot([], [], '-b')  # ,label='Horizontal grid mode')
        # 设置刻度
        self.ax_.set_xticks(np.arange(0, 2 * np.pi, np.pi / 4))
        self.ax_.set_xticklabels(['0', 'π/4', 'π/2', '3π/4', 'π', '5π/4', '3π/2', '7π/4'])

        # # 隐藏y轴坐标
        # self.ax_.set_yticklabels([])

        # 添加标题和标签
        # ax.set_title('Clock Dial with Gaussian Pointer')
        self.ax_.set_xlabel('Head Direction')
        self.fig_.canvas.draw()
        self.fig_.show()

        plt.ion()


    """
    构建高斯分布
    :param x: 自变量
    :param mu: 均值
    :param act_level: 方差的倒数
    """
    def gaussian(self, x, mu, act_level):
        return np.exp(- np.power(x - mu, 2) * act_level / 2.0) / (np.sqrt(2 * math.pi / act_level))

    def showHDCell(self, curr_coordinate):
        if curr_coordinate[1] == 0:
            curr_coordinate[1] = 0.00001

        mu = curr_coordinate[0]
        act_level = curr_coordinate[1]  # act_level = 1/sigma^2

        theta = np.linspace(-np.pi, 3 * np.pi, 1000)

        currHD = self.gaussian(theta, mu, act_level)
        # self.linehl_.set_xdata(theta)
        # self.linehl_.set_ydata(currHD)
        self.linehl_.set_data(theta, currHD)  # 更新数据
        self.ax_.relim()
        self.ax_.autoscale_view()
        self.ax_.set_xlabel(f'Head Direction (θ = {mu * 180 / math.pi : .2f} degree)')
        self.fig_.canvas.draw()
        self.fig_.canvas.flush_events()




if __name__ == "__main__":
    curr_coordinate = [1.14, 100]

    shHD = ClassShowHDCell()
    shHD.showHDCell(curr_coordinate)
    plt.show()
