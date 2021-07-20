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

import numpy as np

import os
import time
from absl import app
from absl import flags

from ravens import tasks
from ravens.dataset import Dataset
from ravens.environments.environment import Environment
import pybullet as p
import pickle


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
        self.homej = np.array([-1, -0.5, 0.5, -0.5, -0.5, 0]) 

    
    def act(self):
        print('please input target joint postion: ')
        x = input().split(',')
        angle = [(float(x[i]) + self.homej[i])* np.pi for i in range(len(x))]
        action = {}
        action['angle'] = angle
        return action
    
    def act_trace(self, x):
        angle = [(float(x[i]) + self.homej[i])* np.pi for i in range(len(x))]
#        action = {}
#        action['angle'] = angle
        return angle


def main(unused_argv):
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
#    rci = RobotControlInterface(env)
    print('Robot control interface finished.')
    episode = []
    reward = 0
    done = False
    trace = []
    f = open('trace', 'ab+')
    a = [-0.05, 0.1, 0.1, -0.3, 0, 0]
    trace.append(a)
    a = [-0.05, 0.12, 0.12, -0.3, 0, 0]
    trace.append(a)
    a = [-0.05, 0.10, 0.1, -0.0, 0, 0]
    trace.append(a)
    a = [-0.05, 0.10, 0.12, 0.1, 0, 0]
    trace.append(a)
    a = [-0.05, 0.1, 0.12, -0.0, 0, 0]
    trace.append(a)
    a = [-0.05, 0.1, 0.12, 0.1, 0, 0]
    trace.append(a)
    a = [0.25, 0.1, 0.10, 0.1, 0, 0]
    trace.append(a)
    
    for t in trace:
        k = agent.act_trace(t)
        env.movej(k)
        for i in range(20):
            p.stepSimulation()
    return 

    while True:
        keys = p.getKeyboardEvents()
        if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_RELEASED:
            print("reset env")
            episode = []
            obs, blocks, targ_pose = env.reset()
        action = agent.act()
#        print('action: ', action)
#        print('please input target joint postion: ')
#        x = input().split(',')
#        angle = [float(x[i]) for i in range(len(x))]
#        action = {}
#        action['angle'] = angle
        
        if action != None:
            episode.append((obs, action, reward, info))

        env.movej(action['angle'])
#        trace.append(action['angle'])
#        a = action['action']
        pickle.dump(action, f)
        for i in range(20):
            p.stepSimulation()
#        obs, reward, done, info = env.step_single(action)
        if done:
            seed += 1
            dataset.add(seed, episode)
            reward = 0
            done = False
            episode = []
            np.random.seed(seed)
            print("episode:{%d}"%seed)
        time.sleep(0.01)


if __name__ == '__main__':
    app.run(main)