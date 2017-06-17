#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
import time


def callback(data):
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

    plt.plot(x, y, 'bo')
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.show()
    time.sleep(0.5)
    plt.close()

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

if __name__ == '__main__':
    listener()
