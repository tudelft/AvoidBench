import os
import rospy
import sys
import math
import numpy as np
from avoid_msgs.msg import Factor
from avoid_msgs.msg import Metrics
import matplotlib.pyplot as plt

class PlotTool():
    def __init__(self):
        self.metrics_sub_ = rospy.Subscriber("/hummingbird/metrics", Metrics, self.metricsCallback)
        self.factors = None
        self.of = []
        self.agv = []
        self.mp = []
        self.pt = []
        self.cn = []
        self.trav = []
        self.rgs = []

    def Save_list2d(self, list1, filename):
        file2 = open(filename + '.txt', 'w')
        for i in range(len(list1)):
            for j in range(len(list1[i])):
                file2.write(str(list1[i][j]))              # write函数不能写int类型的参数，所以使⽤str()转化
                file2.write('\t')                          # 相当于Tab⼀下，换⼀个单元格
            file2.write('\n')                              # 写完⼀⾏⽴马换⾏
        file2.close()

    def Save_list1d(self, list1, filename):
        file2 = open(filename + '.txt', 'w')
        for i in range(len(list1)):
            file2.write(str(list1[i]))              # write函数不能写int类型的参数，所以使⽤str()转化
            file2.write('\t')                          # 相当于Tab⼀下，换⼀个单元格
        file2.close()

    def metricsCallback(self, data):
        self.factors = data.factors
        mission_number = len(self.factors)
        for i in range(mission_number):
            self.of.append(self.factors[i].optimality_factor)
            self.agv.append(self.factors[i].average_goal_velocity)
            self.mp.append(self.factors[i].mission_progress)
            self.pt.append(self.factors[i].processing_time)
            self.cn.append(self.factors[i].collision_number)
            self.trav.append(self.factors[i].traversability)
            self.rgs.append(self.factors[i].relative_gap_size)
        self.Save_list2d(self.of, 'of')
        self.Save_list2d(self.agv, 'agv')
        self.Save_list2d(self.mp, 'mp')
        self.Save_list2d(self.pt, 'pt')
        self.Save_list2d(self.cn, 'cn')
        self.Save_list1d(self.trav, 'trav')
        self.Save_list1d(self.rgs, 'rgs')

        # fig = plt.figure()
        # ax1 = fig.add_subplot(241)
        # ax2 = fig.add_subplot(242)
        # ax3 = fig.add_subplot(243)
        # ax4 = fig.add_subplot(244)
        # ax5 = fig.add_subplot(245)
        # ax6 = fig.add_subplot(246)
        # ax7 = fig.add_subplot(247)
        # ax8 = fig.add_subplot(248)
        # ax1.set(title='Optimality Factor')
        # ax2.set(title='Average Goal Velocity')
        # ax3.set(title='Mission Progress')
        # ax4.set(title='Processing Time')
        # ax5.set(title='Traversability')
        # ax6.set(title='Relative Gap Size')
        # ax7.set(title='Collision Number')
        # ax8.set(title='RGS-Traversability')
        # self.factors = data.factors
        # mission_number = len(self.factors)
        # x = np.linspace(0, mission_number-1, mission_number)
        # print(x)
        # of = np.zeros(mission_number, dtype=np.float64)
        # agv = np.zeros(mission_number, dtype=np.float64)
        # mp = np.zeros(mission_number, dtype=np.float64)
        # pt = np.zeros(mission_number, dtype=np.float64)
        # trav = np.zeros(mission_number, dtype=np.float64)
        # rgs = np.zeros(mission_number, dtype=np.float64)
        # cn = np.zeros(mission_number, dtype=np.float64)
        # for i in range(mission_number):
        #     of[i] = self.factors[i].optimality_factor
        #     agv[i] = self.factors[i].average_goal_velocity
        #     mp[i] = self.factors[i].mission_progress
        #     pt[i] = self.factors[i].processing_time
        #     trav[i] = self.factors[i].traversability
        #     rgs[i] = self.factors[i].relative_gap_size
        #     cn[i] = self.factors[i].collision_number
        # print(of)
        # self.plotFactor(ax1, x, of, 'red', '+')
        # self.plotFactor(ax2, x, agv, 'blue', 'v')
        # self.plotFactor(ax3, x, mp, 'red', 'x')
        # self.plotFactor(ax4, x, pt, 'blue', 'p')
        # self.plotFactor(ax5, x, trav, 'red', 's')
        # self.plotFactor(ax6, x, rgs, 'blue', '*')
        # self.plotFactor(ax7, x, cn, 'red', 'v')
        # ax8.plot(rgs, trav, 'D')
        # plt.show()

    def plotFactor(self, ax, x_data, y_data, color, marker):
        ax.plot(x_data, y_data, color=color, marker=marker, linewidth=1.5)

if __name__ == "__main__":
    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)
    rospy.init_node('test_fly', anonymous=True)
    plot_tool = PlotTool()
    rospy.spin()