#!/usr/bin/env python3
# Software License Agreement (BSD License)
# Auther Yan Li, UTK, 2021
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
    
class ViveRobotBridge:
    def __init__(self, id='/vive/controller_LHR_FFFF3F47/joy', topic='tarpos_pub'):
        self.offset = [0,0,0]
        self.offset_flag = 0
        self.reset_pose_flag = 0
        self.reset_to_init_flag = 0
        self.pose_flag = 0
        self.pub = rospy.Publisher(topic, geometry_msgs.msg.Transform,
                                   queue_size=1)
#        print("subscribe controller button events")
        self._joy_sub = rospy.Subscriber(id, Joy, self.vive_controller_button_callback,
                                         queue_size=1)
#        self.tf_thread = threading.Thread(target = self.TransformListener, args=())
#        print("starting thread")
#        self.tf_thread.start()

    def vive_controller_button_callback(self, msg):
        if msg.buttons[1] == 1:
            print("controller events")
            self.offset_flag = 1
        else:
            self.offset_flag = 0
        
        # reset the robot pose to init pose
        if msg.buttons[0] == 1:
#            print("controller events")
            self.reset_to_init_flag = 1
        else:
            self.reset_to_init_flag = 0
        
        # reset the rotation to downwards
        if msg.buttons[2] == 1:
#            print("controller events")
            self.reset_pose_flag = 1
        else:
            self.reset_pose_flag = 0
        
        if msg.buttons[3] == 1:
#            print("controller events")
            self.pose_flag = 1
        else:
            self.pose_flag = 0
        
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

if __name__ == '__main__':

    vrb = ViveRobotBridge('/vive/controller_LHR_FFFF3F47/joy')
    vrb_orientation = ViveRobotBridge('/vive/controller_LHR_FFF43D45/joy','tarori_pub')
#    vrb.__init__()
    
    rospy.init_node('vive_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
#    pub = rospy.Publisher('tarpos_pub', geometry_msgs.msg.Transform, queue_size=1)
#    _joy_sub = rospy.Subscriber('/vive/controller_LHR_FFFF3F47/joy', Joy, vive_controller_button_callback, queue_size=1)
    
    temp_flag = 0
    
    pre_position = [0,0,0]
    pre_pose = [1, 0, 0, 0]
    temp = [1, 0, 0, 0]

    rate = rospy.Rate(10.0)
    
    limits = 0.05
    
    msg = geometry_msgs.msg.Transform()  
    msg_ori = geometry_msgs.msg.Transform()  
    msg_ori.rotation.x = 0
    msg_ori.rotation.y = 0
    msg_ori.rotation.z = 0
    msg_ori.rotation.w = 1
    
#    msg_ori = geometry_msgs.msg.Transform()  
#    msg_ori.translation.x = 0
#    msg_ori.translation.y = 0
#    msg_ori.translation.z = 0
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'translation_tf', rospy.Time())
#            rospy.loginfo(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("error")
            rate.sleep()
            continue
        

        if vrb.offset_flag == 1:
            if temp_flag == 0:
                pre_position[0]=trans.transform.translation.x
                pre_position[1]=trans.transform.translation.y
                pre_position[2]=trans.transform.translation.z
                
            else:
#                compute delta distance
                x = trans.transform.translation.x-pre_position[0]
                y = trans.transform.translation.y-pre_position[1]
                z = trans.transform.translation.z-pre_position[2]
                
                r = np.sqrt(x*x + y*y + z*z)
                
                if (r < 0.01):
                    continue
                
                msg.translation.x = x
                msg.translation.y = y
                msg.translation.z = z
                
                
                msg.rotation.x = msg_ori.rotation.x
                msg.rotation.y = msg_ori.rotation.y
                msg.rotation.z = msg_ori.rotation.z
                msg.rotation.w = msg_ori.rotation.w
                msg = InBound(msg, limits)                
                print(msg.translation, msg.rotation)                
                vrb.pub.publish(msg)
                
                pre_position[0]=trans.transform.translation.x
                pre_position[1]=trans.transform.translation.y
                pre_position[2]=trans.transform.translation.z
        
        if vrb.pose_flag == 1:
            msg_ori.rotation.x = trans.transform.rotation.x
            msg_ori.rotation.y = trans.transform.rotation.y
            msg_ori.rotation.z = trans.transform.rotation.z
            msg_ori.rotation.w = trans.transform.rotation.w
                    
        if vrb.reset_pose_flag == 1:
            msg_ori.rotation.x = 0
            msg_ori.rotation.y = 0
            msg_ori.rotation.z = 0
            msg_ori.rotation.w = 1
        
        if vrb.reset_to_init_flag == 1:
#                compute delta distance
            msg.translation.x = 0
            msg.translation.y = 0
            msg.translation.z = 0
            
            msg.rotation.x = 0
            msg.rotation.y = 0
            msg.rotation.z = 0
            msg.rotation.w = 1
            msg = InBound(msg, limits)                
            print('Reset Robot')                
            vrb.pub.publish(msg)
            
            
        temp_flag = vrb.offset_flag
        
                
        rate.sleep()
#        if vrb_orientation.offset_flag == 1:
##        print(vrb.offset_flag)    
##            msg.translation.x = trans.transform.translation.x-vrb.offset[0]
##            msg.translation.y = trans.transform.translation.y-vrb.offset[1]
##            msg.translation.z = trans.transform.translation.z-vrb.offset[2] + 0.8
#            msg.rotation.x = trans_ori.transform.rotation.x
#            msg.rotation.y = trans_ori.transform.rotation.y
#            msg.rotation.z = trans_ori.transform.rotation.z
#            msg.rotation.w = trans_ori.transform.rotation.w
#
#            vrb_orientation.publish(msg)

        
#    