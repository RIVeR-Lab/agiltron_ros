#! /usr/bin/python3
'''
Author: Nathaniel Hanson
Date: 12/10/2021
File: switcher.py

Purpose: Enable switching between fiber optic channels in the Agiltron Fiber-Fiber switcher
'''

# Future proofing python 2
from __future__ import nested_scopes
from __future__ import generators
from __future__ import division
from __future__ import absolute_import
from __future__ import with_statement
from __future__ import print_function
# Standard package imports
import rospy
import sys
import os
import cv2
import roslib
import math
import traceback
import numpy as np
import serial
from std_msgs.msg import UInt8, Empty


class AGILTRON:
    def __init__(self):
        # Create serial connection
        self.conn = serial.Serial('/dev/ttyUSB0')
        # Send the fiber to position
        self.go_home(Empty())
        self.now = rospy.get_time()
        # Setup subscribers for pertinent topics
        rospy.Subscriber('/agiltron/read_fiber', UInt8, self.do_switch)
        rospy.Subscriber('/agiltron/home', Empty, self.go_home)

    def go_home(self, msg):
        '''
        Send MEMS mirror to home position
        '''
        rospy.loginfo(f'Agiltron Info: Returning to home position')
        comm = bytearray(b'\x01\x20\x00\x00')
        resp = self.conn.write(comm)
        # Update last move time
        self.now = rospy.get_time()
    
    def do_switch(self, msg):
        '''
        Algin mirror to read from correct fiber
        '''
        #print(msg.data)
        # Grab current time to limit how often we switch
        channel = int(msg.data)
        if rospy.get_time()- self.now < 2:
            rospy.logwarn(f'Agiltron Warning: Requesting mirror switch too quickly!')
        if 1 <= channel <= 16:
            comm = bytearray(b'\x01\x12\x00')
            comm.append(channel)
            rospy.loginfo(f'Agiltron Info: Switching to fiber {channel}')
            resp = self.conn.write(comm)
            self.now = rospy.get_time()
            if resp != 4:
                rospy.logerr(f'Agiltron Error: Unable to switch to selected channel, response was: {resp}')
        else:
            rospy.logerr(f'Agiltron Error: Fiber {channel} is invalid. Must be in range [1,16]')

if __name__ == '__main__':
    try:
        rospy.init_node('agiltron', anonymous=True)
        processor = AGILTRON()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Fiber switching node failed!')
        pass