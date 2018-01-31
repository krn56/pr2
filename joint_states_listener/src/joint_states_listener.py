#!/usr/bin/env python
# -*- coding: utf-8 -*-

# spins off a thread to listen for joint_states message
# and provides the same information as a service

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from joint_states_listener.srv import *
from sensor_msgs.msg import JointState
import threading

# holds the latest states obtained from joint_states messages

class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.service('return_joint_states', ReturnJointStates, self.return_joint_states)

    # thread function : listen for joint_states message
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()

    # callback function : when a joint_states message arrives, save the values
    def joint_states_callback(





# run the server
if __name__=="__main__":
    latestjointstates = LatestJointStates()

    print "joint_states_listener server started, waiting for queries"
    rospy.spin()
