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
import geometry_msgs.msg
import tf
import tf2_ros
import numpy as np

from pyquaternion import Quaternion
from tf.transformations import quaternion_matrix

from sensor_msgs.msg import Joy

    
    
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


if __name__ == '__main__':

    vrb = ViveRobotBridge('/vive/controller_LHR_FFFF3F47/joy')    
    rospy.init_node('vive_traslation')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)    
    br = tf.TransformBroadcaster()    
    translation = [0, 0, 0.25]
    
    while not rospy.is_shutdown():
#        try:
#            trans = tfBuffer.lookup_transform('world', 'controller_LHR_FFFF3F47', rospy.Time())
#        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#            rospy.loginfo("error")
#            rate.sleep()
#            continue

        br.sendTransform((0,0,0.2),
             (0.707, -0.707, 0, 0),
             rospy.Time.now(),
             'translation_tf',
             "controller_LHR_FFFF3F47")