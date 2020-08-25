#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
import matplotlib.pyplot as plt
import json
import math
import time
import os
from datetime import datetime

class Avoidance():
    def __init__(self):
        self.obj = []
        self.VDATA = []
        self.new_obj = 0
        self.new_VDATA = 0
    def subscribe_obj(self, msg):
        self.obj = np.array(msg.data)
        if self.obj.shape[0] > 0:
          self.obj.resize((int(self.obj.shape[0]/5), 5))
        self.obj.tolist()
        self.new_obj = 1
    def subscribe_VDATA(self, msg):
        self.VDATA = list(msg.data)
        self.new_VDATA = 1

node_name = 'My_Test'
rospy.init_node(node_name)
avoid = Avoidance()
rospy.Subscriber('OBSTT', Float64MultiArray, avoid.subscribe_obj)
rospy.Subscriber('VDATA', Float64MultiArray, avoid.subscribe_VDATA)
pub = rospy.Publisher('PCDAT', Float64MultiArray, queue_size=10)
rate = rospy.Rate(20)

p_rob = []
p_obs = []
detected = []
len_path = 0
map_out = '/home/nguyen/hien_ws/map_out'

while not rospy.is_shutdown():
    if len_path == 0: print [node_name], 'Waiting to read map...'
    while len_path == 0 and not rospy.is_shutdown():
        if os.path.isfile(map_out):
            json_input = open(map_out)
            input_path = json.load(json_input)
            print [node_name], 'Read the map successfully!'
            len_path = len(input_path)
            # for i in range(len_path):
            #     print input_path[i]
            print [node_name], 'Length of the map:', len_path
            final_goal = [input_path[-1]['x'], input_path[-1]['y']]
            i_finalgoal = len_path - 1 
    pre_obs = np.copy(detected)
    robot_pos = []
    while len(robot_pos) == 0 and not rospy.is_shutdown():
        if avoid.new_VDATA == 1 and avoid.new_obj == 1:
            robot_pos = [avoid.VDATA[0], avoid.VDATA[1]]
            i_curr = int(avoid.VDATA[3])
            robot_yaw = avoid.VDATA[2] 
            avoid.new_VDATA = 0
            detected = avoid.obj
            len_detected = len(detected)
            print 'Type of detected:', type(detected)
            avoid.new_obj = 0
    if rospy.is_shutdown():
        continue 
    if p_rob == [] and p_obs == []:
        fig, ax = plt.subplots(1)
        plt.grid(True)
        plt.axis("equal")
        for i in range(len_path):
            plt.plot(input_path[i]['x'], input_path[i]['y'], "-y")
        p_goal = plt.plot(final_goal[0], final_goal[1], "ob")
        append = p_obs.append
        for obstacle in detected:
            append(plt.plot(obstacle[0], obstacle[1], "og"))
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='g', alpha=0.5, edgecolor='None'))
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='y', alpha=0.5, edgecolor='None'))
    else:
        # p_rob[0].remove()
        for i in range(len(p_obs)):
            p_obs[i][0].remove()
        p_obs = []
        for obstacle in pre_obs:
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='w', alpha=1, edgecolor='w'))
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='w', alpha=1, edgecolor='w'))
        append = p_obs.append
        for obstacle in detected:
            append(plt.plot(obstacle[0], obstacle[1], "og"))
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='g', alpha=0.5, edgecolor='None'))
            ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='y', alpha=0.5, edgecolor='None'))
    p_rob = plt.plot(robot_pos[0], robot_pos[1], "or")
    plt.pause(0.1)