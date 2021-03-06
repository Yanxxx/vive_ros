#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Author: Yan Li, UTK, 2021
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from std_msgs.msg import String
import rospy
import math
import geometry_msgs.msg
import tf
import roslib
import tf2_ros
import numpy as np

from pyquaternion import Quaternion

#import sensor_msgs.msg
from sensor_msgs.msg import Joy

import threading 


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
    
    
class ViveRobotBridge:
    def __init__(self, id='/vive/controller_LHR_FFFF3F47/joy', topic='tarpos_pub'):
        self.offset = [0,0,0]
        self.offset_flag = 0
        self.pub = rospy.Publisher(topic, geometry_msgs.msg.Transform,
                                   queue_size=1)
        self._joy_sub = rospy.Subscriber(id, Joy, self.vive_controller_button_callback,
                                         queue_size=1)

    def vive_controller_button_callback(self, msg):
        if msg.buttons[1] == 1:
#            print("controller events")
            self.offset_flag = 1
        else:
            self.offset_flag = 0
        
    def publish(self, msg):
        self.pub.publish(msg)

def InBound(msg, limits):       
    if msg.translation.x >limits:
        msg.translation.x = limits                
    if msg.translation.y >limits:
        msg.translation.y = limits                
    if msg.translation.z >limits:
        msg.translation.z = limits                     
    if msg.translation.x <-limits:
        msg.translation.x = -limits                
    if msg.translation.y <-limits:
        msg.translation.y = -limits                
    if msg.translation.z <-limits:
        msg.translation.z = -limits
    return msg

def TransformedPosition(translation, rotation):
    controller_offset = [0, 0, 0.2]
    
    Q = [rotation.x, rotation.y,
         rotation.z, rotation.w]
    R = quaternion_rotation_matrix(Q)
    
#    print('original translation: ', translation.x, 
#          translation.y, translation.z)
    
    rotated_offset = np.inner(R, np.array(controller_offset))
    
#    print('rotated_offset: ', rotated_offset)
    result = [translation.x + rotated_offset[0],
            translation.x + rotated_offset[1],
            translation.x + rotated_offset[2]]
#    print (result)
    return result

if __name__ == '__main__':
    print('modified controller talker is started: ')
    vrb = ViveRobotBridge('/vive/controller_LHR_FFFF3F47/joy')
#    vrb.__init__()
    
    rospy.init_node('vive_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    temp_flag = 0    
    pre_position = [0,0,0]
    current_position = [0,0,0]
    pre_pose = [1, 0, 0, 0]
    temp = [1, 0, 0, 0]

    rate = rospy.Rate(10.0)    
    limits = 0.02
    
    msg = geometry_msgs.msg.Transform()
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'controller_LHR_FFFF3F47', rospy.Time())
#            rospy.loginfo(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("error")
            rate.sleep()
            continue
        

        if vrb.offset_flag == 1:
            if temp_flag == 0:              
                pre_position = TransformedPosition(trans.transform.translation,
                                                   trans.transform.rotation)
            else:                
                current_position = TransformedPosition(trans.transform.translation,
                                                   trans.transform.rotation)                           
                msg.translation.x = current_position[0]-pre_position[0]
                msg.translation.y = current_position[1]-pre_position[1]
                msg.translation.z = - (current_position[2]-pre_position[2])
                
                msg.rotation.x = trans.transform.rotation.x
                msg.rotation.y = trans.transform.rotation.y
                msg.rotation.z = trans.transform.rotation.z
                msg.rotation.w = trans.transform.rotation.w
                
                print('delta position: ')
                print(msg.translation)
#                msg = InBound(msg, limits)                
                vrb.pub.publish(msg)
                pre_position = current_position
            
        temp_flag = vrb.offset_flag
        
                
        rate.sleep()