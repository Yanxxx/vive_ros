#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import os
import time
from absl import app
from absl import flags

from ravens import tasks
from ravens.dataset import Dataset
from ravens.environments.environment import Environment
import pybullet as p

from pyquaternion import Quaternion

#import sensor_msgs.msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

import threading 
import PyKDL as kdl

from matplotlib import pyplot as plt



flags.DEFINE_string('assets_root', '.', '')
flags.DEFINE_string('data_dir', '.', '')
flags.DEFINE_bool('disp', False, '')
flags.DEFINE_bool('shared_memory', False, '')
flags.DEFINE_string('task', 'towers-of-hanoi', '')
flags.DEFINE_string('mode', 'train', '')
flags.DEFINE_integer('n', 1000, '')

assets_root = "/home/yan/git/ravens/ravens/environments/assets/"
dataset_root = "/data/ravens_demo/"
#task_name = "place-red-in-green"
#task_name = "block-insertion-nofixture"
task_name = "auto-excavation"
mode = "train"
FLAGS = flags.FLAGS

    
class teleop_agent:
    def __init__(self, env) -> None:
        self.env = env
        self.delta_pos = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]        
        self.vive_controller_pos_sub = rospy.Subscriber('ravens_teleop', 
                                         geometry_msgs.msg.Transform,
                                         self.vive_controller_pos_callback,
                                         queue_size=1)
        self.update = False
        
    def vive_controller_pos_callback(self, msg):
#        print('topic from ros')
        self.update = True
        self.delta_pos[0] = msg.translation.x
        self.delta_pos[1] = msg.translation.y
        self.delta_pos[2] = msg.translation.z

        self.orientation[0] = msg.rotation.x
        self.orientation[1] = msg.rotation.y
        self.orientation[2] = msg.rotation.z
        self.orientation[3] = msg.rotation.w
        
    
    def act(self):
        if not self.update:
            return None
        self.update = False
        s = p.getLinkState(self.env.ur5, self.env.ee_tip)
#        print(s)
#        input('for debug')
        ee_position_ref = list(s[4])
        ee_position = [ee_position_ref[i] + self.delta_pos[i] \
                       for i in range(len(self.delta_pos))]
#        ee_orientation = self.orientation
#        ee_orientation = s[5]
        ee_orientation = [1,0,0,0]
        print(self.delta_pos, ee_orientation)

        action = {}
        self.pose = (tuple(ee_position), tuple(ee_orientation))
        action['pose'] = (tuple(ee_position), tuple(ee_orientation))
        action['grasp'] = (0,0)
        return action


def main(unused_argv):
    rospy.init_node('raven_vive_teleop')    
    pre_position = [0,0,0]
    pre_pose = [1, 0, 0, 0]
    temp = [1, 0, 0, 0]
    # same loop rate as gym environment
    rate = rospy.Rate(60.0)    
    env = Environment(
      assets_root,
      disp=True,
      hz=60)    
    task = tasks.names[task_name]()
    task.mode = mode    
    env.set_task(task)
    obs, blocks, targ_pose = env.reset()
    print('reset environment finished.')
    info = None
    agent = teleop_agent(env)
    dataset = Dataset(os.path.join(dataset_root, f'ravens_demo-{time.time_ns()}'))
    print('Dataset setup finished.')
    seed = 0
    br = tf.TransformBroadcaster()
#    rci = RobotControlInterface(env)
    print('Robot control interface finished.')
    episode = []
    reward = 0
    done = False

    while not rospy.is_shutdown():
        keys = p.getKeyboardEvents()
        if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_RELEASED:
            print("reset env")
            episode = []
            obs, blocks, targ_pose = env.reset()
        action = agent.act()
#        print('action: ', action)
        if action != None:
            episode.append((obs, action, reward, info))

        obs, reward, done, info = env.step_single(action)
        if done:
            seed += 1
            dataset.add(seed, episode)
            reward = 0
            done = False
            episode = []
            np.random.seed(seed)
            print("episode:{%d}"%seed)
        rate.sleep()


if __name__ == '__main__':
    app.run(main)