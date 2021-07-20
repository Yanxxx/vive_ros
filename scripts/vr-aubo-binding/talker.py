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
#roslib.load_manifest('learning_tf')
#
#
#offset = [0, 0 , 0]
#
#def talker1():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker1', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()
#
#def talker():
#    print("run talker")
#    rospy.init_node('talker1', anonymous=True)
#    pub = rospy.Publisher('tarpos_pub', geometry_msgs.msg.Twist,queue_size=1)
#    rate = rospy.Rate(10) #10Hz
#    phase = 0
#    x = 0.2
#    cmd = geometry_msgs.msg.Twist()
#    cmd.linear.x = x
#    cmd.angular.x = 0.0
#    cmd.angular.y = 3.14
#    cmd.angular.z = 0.0
#    while not rospy.is_shutdown():
##        hello_str = "hello world %s" % rospy.get_time()
#        y = 0.2 * math.sin(phase)
#        z = 0.2 * math.cos(phase)
#        cmd.linear.y = y
#        cmd.linear.z = z
##        cmd.angular.z = angular
#        rospy.loginfo(cmd)
#        pub.publish(cmd)
#        phase += 0.1
#        rate.sleep()
#        
#def callback1(data):
#    print("data received!")
##    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
##    rospy.loginfo(data)
##    pub = rospy.Publisher('tarpos_pub', geometry_msgs.msg.Twist,queue_size=1)
##    try:
##        (trans,rot) = listener.lookupTransform('/controller_LHR_FFF43D45', '/controller_LHR_FFFF3F47', rospy.Time(0))
##    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
##        pass
##    cmd = geometry_msgs.msg.Twist()
##    cmd.angular.z = trans[0]
##    pub.publish(cmd)
#
#    
#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard something...AnyThing?")
#    
#def topic_subscriber():
#    rospy.Subscriber("/tf", rospy.AnyMsg, callback)
#
#def transform_listener():
#    tf2_buffer = tf2_ros.Buffer()
#    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
#          
##    while not (tf2_buffer.can_transform('controller_LHR_FFFF3F47', 'world', rospy.Time.now())):
##        pass
#    
#    transform = tf2_buffer.lookup_transform('controller_LHR_FFFF3F47', 'world', rospy.Time.now())
#
#    print('I will not get this transform: {}'.format(transform))
#    
#
#def listener1():
#
#    # In ROS, nodes are uniquely named. If two nodes with the same
#    # name are launched, the previous one is kicked off. The
#    # anonymous=True flag means that rospy will choose a unique
#    # name for our 'listener' node so that multiple listeners can
#    # run simultaneously.
#    rospy.init_node('vive_tf_listener', anonymous=True)
#    listener = tf.TransformListener()
#    
#    listener.waitForTransform("/tf")
#    
#
#    rospy.Subscriber("tf", listener, callback1)
#
#    # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()
#        
        
#if __name__ == '__main__':
#    rospy.init_node('vive_tf_listener', anonymous=True)
##    topic_subscriber()
#    transform_listener()
#    rospy.spin()

#    rospy.init_node('vive_tf_listener')
#    listener = tf.TransformListener()
#    
#    pub = rospy.Publisher('tarpos_pub', geometry_msgs.msg.Twist,queue_size=1)    
#    rate = rospy.Rate(10.0)
#    while not rospy.is_shutdown():
#        (trans,rot) = listener.lookupTransform('/world_vive', '/controller_LHR_FFF43D45',rospy.Time(0))
#        
##        try:
##            (trans,rot) = listener.lookupTransform('/controller_LHR_F,FF43D45', '/controller_LHR_FFFF3F47', rospy.Time(0))
##        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
##            pass
#        cmd = geometry_msgs.msg.Twist()
#        cmd.angular.z = rot[2]
#        cmd.angular.x = rot[0]
#        cmd.angular.y = rot[1]
#        cmd.linear.z = trans[2]
#        cmd.linear.x = trans[0]
#        cmd.linear.y = trans[1]
#        pub.publish(cmd)
#        rospy.loginfo(cmd)
#
#        rate.sleep()

#    try:
#        listener()
#    except rospy.ROSInterruptException:
#        pass
    
#def toEulerAngles(q)    :
#    # rool
#    sinr_cosp = 2 * (q[3]*q[0]+q[1]*q[2])
#    cosr_cosp = 1 - 2 * (q[0]*q[0]+q[1]*q[1])
#    angles_roll = math.atan2(sinr_cosp, cosr_cosp)
#    
#    # pitch 
#    sinp = 2 * (q[3]*q[1]-q[2]*q[0])
#    if abs(sinp) > 1:
#        if sinp>=0:
#            angles_pitch = math.pi/2
#        else:
#            angles_pitch = -math.pi/2
#    else:
#        angles_pitch = math.asin(sinp)
#            
#    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
#    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
#    
#    angles_yaw = math.atan2(siny_cosp, cosy_cosp)
#    
#    return [angles_roll, angles_pitch, angles_yaw]


#global _offset_flag 

#def vive_controller_button_callback(msg):
#    if msg.buttons[1] == 1:
#        _offset_flag = 1
#    else:
#        _offset_flag = 0
#        print("button pressed")
        
    
    
class ViveRobotBridge:
    def __init__(self, id='/vive/controller_LHR_FFFF3F47/joy', topic='tarpos_pub'):
        self.offset = [0,0,0]
        self.offset_flag = 0
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
    
    limits = 0.02
    
    msg = geometry_msgs.msg.Transform()  
    msg.rotation.x = 0
    msg.rotation.y = 0
    msg.rotation.z = 0
    msg.rotation.w = 1
    
    msg_ori = geometry_msgs.msg.Transform()  
    msg_ori.translation.x = 0
    msg_ori.translation.y = 0
    msg_ori.translation.z = 0
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'controller_LHR_FFFF3F47', rospy.Time())
            trans_ori = tfBuffer.lookup_transform('world', 'controller_LHR_FFF43D45', rospy.Time())
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
                msg.translation.x = trans.transform.translation.x-pre_position[0]
                msg.translation.y = trans.transform.translation.y-pre_position[1]
                msg.translation.z = trans.transform.translation.z-pre_position[2]
                
                msg = InBound(msg, limits)                
                print(msg.translation)                
                vrb.pub.publish(msg)
                
                pre_position[0]=trans.transform.translation.x
                pre_position[1]=trans.transform.translation.y
                pre_position[2]=trans.transform.translation.z
            
        temp_flag = vrb.offset_flag
        
        if vrb_orientation.offset_flag == 1:
            msg_ori.rotation.x = trans_ori.transform.rotation.x
            msg_ori.rotation.y = trans_ori.transform.rotation.y
            msg_ori.rotation.z = trans_ori.transform.rotation.z
            msg_ori.rotation.w = trans_ori.transform.rotation.w
            print(msg_ori.rotation)                
            vrb_orientation.pub.publish(msg_ori)
                
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