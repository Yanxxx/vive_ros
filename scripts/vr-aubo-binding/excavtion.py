#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 19 15:57:46 2021

@author: yan
"""



# coding=utf-8
# Copyright 2021 Yan Li, UTK, Knoxville, TN, 37996.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Data collection script."""

import os

from absl import app
from absl import flags

import numpy as np

from ravens import tasks
from ravens.dataset import Dataset
from ravens.environments.environment import Environment
from std_msgs.msg import String
import rospy
import math
import geometry_msgs.msg
import tf
import roslib
import tf2_ros
import numpy as np

from pyquaternion import Quaternion

from sensor_msgs.msg import Joy
#
#flags.DEFINE_string('assets_root', '.', '')
#flags.DEFINE_string('data_dir', '.', '')
#flags.DEFINE_bool('disp', False, '')
#flags.DEFINE_bool('shared_memory', False, '')
#flags.DEFINE_string('task', 'towers-of-hanoi', '')
#flags.DEFINE_string('mode', 'train', '')
#flags.DEFINE_integer('n', 1000, '')
#
#FLAGS = flags.FLAGS

flags.DEFINE_string('assets_root', '/home/yan/git/ravens/ravens/environments/assets/', '')
flags.DEFINE_string('data_dir', '/data/ravens_demo/', '')
flags.DEFINE_bool('disp', 'True', '')
flags.DEFINE_bool('shared_memory', False, '')
flags.DEFINE_string('task', 'auto-excavation', '')
flags.DEFINE_string('mode', 'test', '')
flags.DEFINE_integer('n', 10, '')

#assets_root = "/home/yan/git/ravens/ravens/environments/assets/"
#dataset_root = "/data/ravens_demo/"
##task_name = "place-red-in-green"
##task_name = "block-insertion-nofixture"
#task_name = "auto-excavation"
#mode = "train"
FLAGS = flags.FLAGS


#homej = [-1, -0.5, 0.5, -0.5, -0.5, 0]
#def val2robot(val):
#    targ_j = None
#    if len(val) == 6:
#        targ_j = np.array(homej) + np.array(val)
#        targ_j *= np.pi
#    if len(val) == 7:
#        targ_pos = val[:3]
#        targ_pose = val[3:]
#        targ_j = (targ_pos, targ_pose)
#    return targ_j

    
class ViveRobotBridge:
    def __init__(self, id='/vive/controller_LHR_FFFF3F47/joy', topic='tarpos_pub'):
        self.offset = [0,0,0]
        self.offset_flag = 0
        self.pose_lock = 0
        self.reset = 0
        self.home = 0
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
        if msg.buttons[2] == 1:
#            print("controller events")
            self.pose_lock = 1
        else:
            self.pose_lock = 0
        if msg.buttons[0] == 1:
#            print("controller events")
            self.reset = 1
        else:
            self.reset = 0
        if msg.buttons[3] == 1:
#            print("controller events")
            self.home = 1
        else:
            self.home = 0
        
    def publish(self, msg):
        self.pub.publish(msg)
        
class teleop_agent:
    def __init__(self, env) -> None:
        self.env = env
        self.delta_pos = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]        
        self.vive_controller_pos_sub = rospy.Subscriber('vive_traslation', 
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

  # Initialize environment and task.
  env = Environment(
      FLAGS.assets_root,
      disp=FLAGS.disp,
      shared_memory=FLAGS.shared_memory,
      hz=480)
#  print(tasks.names['excavation'])
  task = tasks.names[FLAGS.task]()
  task.mode = FLAGS.mode

  # Initialize scripted oracle agent and dataset.
#  agent = task.oracle(env)
  dataset = Dataset(os.path.join(FLAGS.data_dir, f'{FLAGS.task}-{task.mode}'))

  # Train seeds are even and test seeds are odd.
  seed = dataset.max_seed
  if seed < 0:
    seed = -1 if (task.mode == 'test') else -2
    


#  episode, total_reward = [], 0
  seed += 2
  np.random.seed(seed)
  env.set_task(task)
  obs, blocks, targ_pose = env.reset()
#  obs = env.reset()
  act = {}
  
  vrb = ViveRobotBridge('/vive/controller_LHR_FFFF3F47/joy', 'ravens_teleop')
#    vrb.__init__()

  rospy.init_node('vive_listener')  
  rate = rospy.Rate(10.0)

  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  temp_flag = 0
  pre_position = [0,0,0]
  tmp_cur = [0,0,0]
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
#  ((0.46562498807907104, -0.375, 0.3599780201911926), 
#                        (0.0, 0.0, 0.0, 1.0))
  home_position = [0.46562498807907104, 0.1, 0.35]
  current_position = np.array([0.46562498807907104, 0.1, 0.35])
  delta = [0, 0, 0]
  home_pose = [0,0,0,1]
  pose = [0,0,0,1]
  pose_lock = 0

  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform('world', 'translation_tf', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.loginfo("error")
      rate.sleep()
      continue    
    
#    print(vrb.offset_flag, rospy.is_shutdown())
      
    if vrb.home == 1:
      hp = (home_position, home_pose)
      act['pose'] = hp
      act['grasp'] = [0, 0]
      env.step_move_ex(act)

    if vrb.reset == 1:
      print('reset enviroment')
      obs, blocks, targ_pose = env.reset()
    if vrb.pose_lock == 1:
      pose_lock = (pose_lock + 1) % 2
      if pose_lock == 1:
          print('rotation locked')
      elif pose_lock == 0:
          print('rotation unlocked')
#      act['pose'] = hp
#      act['grasp'] = [0, 0]
#      env.step_move_ex(act)
    
    if vrb.offset_flag == 1:
      if temp_flag == 0:
        pre_position[0]=trans.transform.translation.x
        pre_position[1]=trans.transform.translation.y
        pre_position[2]=trans.transform.translation.z            
      else:
        delta[0] = trans.transform.translation.x-pre_position[0]
        delta[1] = trans.transform.translation.y-pre_position[1]
        delta[2] = trans.transform.translation.z-pre_position[2]  
        
        tmp_cur[0]=trans.transform.translation.x
        tmp_cur[1]=trans.transform.translation.y
        tmp_cur[2]=trans.transform.translation.z
        if pose_lock == 0:
          pose[0] = trans.transform.rotation.x
          pose[1] = trans.transform.rotation.y
          pose[2] = trans.transform.rotation.z
          pose[3] = trans.transform.rotation.w
#        print(pose)      
        
#        print(pre_position, tmp_cur, delta)
#        print(delta)
        if sum(abs(np.array(delta))) < 0.01:
            continue
        delta[1] *= -1
        current_position += delta        
        targ = (current_position, pose)   
        pre_position[0]=trans.transform.translation.x
        pre_position[1]=trans.transform.translation.y
        pre_position[2]=trans.transform.translation.z
        act['pose'] = targ
        act['grasp'] = [0, 0]
#        print('robot move')
        print(targ)
        env.step_move_ex(act)
#        print('robot move end')
    
    temp_flag = vrb.offset_flag
    rate.sleep()

  print('end')


if __name__ == '__main__':
  app.run(main)
