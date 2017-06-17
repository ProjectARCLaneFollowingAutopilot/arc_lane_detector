#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

x = []
y = []
fig = plt.figure()
subplt = fig.add_subplot(1, 1, 1)

def callback(data):
    print("Called")
    rcv = data.data.split(" ")
    left_bot_x = float(rcv[0])
    left_bot_y = float(rcv[1])
    left_top_x = float(rcv[2])
    left_top_y = float(rcv[3])
    right_bot_x = float(rcv[4])
    right_bot_y = float(rcv[5])
    right_top_x = float(rcv[6])
    right_top_y = float(rcv[7])
    x = [left_bot_x, left_top_x, right_bot_x, right_top_x]
    y = [left_bot_y, left_top_y, right_bot_y, right_top_y]
    ani = animation.FuncAnimation(fig, animate, interval = 1000)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plot_lines', anonymous = True)

    rospy.Subscriber('lane_boundaries', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def animate(i):
    print("Called2")
    subplt.plot(x, y)

axes = plt.gca()
axes.set_xlim([-20,20])
axes.set_ylim([0,30])
plt.show()


if __name__ == '__main__':
    listener()
