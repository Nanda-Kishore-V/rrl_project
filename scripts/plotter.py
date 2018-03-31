import matplotlib.pyplot as plt

from constants import (
    X,
    Y,
)

class Plotter():

    def __init__(self):
        self.fig = plt.figure()
        # plt.axis('equal')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim([0, X])
        self.ax.set_ylim([0, Y])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.plot = None
        self.timeAnnotation = self.ax.annotate("Time", xy=(0, 0), xycoords='axes fraction', fontsize=12, ha='right', va='bottom')

    def plotting(self, x, y, c, s=1):
        if self.plot == None:
            self.plot = self.ax.scatter(x, y, c=c, s=s)
        else:
            self.ax.scatter(x, y, c=c, s=s)

    def plot_square(self, x, y, c, s):
        if self.plot == None:
            self.plot = self.ax.scatter(x, y, c=c, marker=(4,0,0), s=s)
        else:
            self.ax.scatter(x, y, c=c, marker=(4,0,0), s=s)

    def arrow(self, x, y, theta):
        self.ax.quiver(x, y, math.cos(theta), math.sin(theta), units='width')

    def curve(self, x_axis, y_axis, c):
        self.ax.plot(x_axis, y_axis, c=c)

    def sleep(self, time):
        plt.pause(time)

    def mark_time(self, time):
        self.timeAnnotation.set_text("{} s".format(time))

    def draw_start_end(self, robot):
        self.plotting(robot.pose[0], robot.pose[1], robot.color, 50)
        self.plot_square(robot.goal[0], robot.goal[1], robot.color, 50)

    def close(self, type):
        plt.close(type)
