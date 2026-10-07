#! /usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory',anonymous=True)
joint_positions_all = []


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    global joint_positions_all
    joint_positions_all.append(data.position)
    
rospy.Subscriber("joint_states", JointState, callback)


while not rospy.is_shutdown():
	rospy.spin()

with open('/home/felix/Desktop/SAE_IUT/joint_positions.csv','w') as csv_file:
    writer = csv.writer(csv_file)
    for line in joint_positions_all:
        writer.writerow(line)










