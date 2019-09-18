import zmq 
import sys 
import argparse
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose
import msg.world_control_pb2 as GZ_world_control

import wrapped_gazebo_prius as simulator
import environment as En

from coloredPrint import *
from net import *

import cv2
import time
import math
import signal
import random
import threading
import numpy as np
import tensorflow as tf
from collections import deque

# param definition
GAME = 'angry-car' # the name of the game being played for log files
ACTIONS = 15 # number of valid actions
INITIAL_EPSILON = 0.0001 # starting value of epsilon
OBSERVE_EPSILON = 0.5 # starting value of epsilon
INITIAL_EPSILON = 0.2 # starting value of epsilon
OBSERVE = 8000. # timesteps to observe before training
REPLAY_MEMORY = 30000 # number of previous transitions to remember
EXPLORE = 3000000. # frames over which to anneal epsilon
#EXPLORE = 2000000. # frames over which to anneal epsilon
FINAL_EPSILON = 0.005 # final value of epsilon
GAMMA = 0.96 # decay rate of past observations
BATCH = 32
INITIAL_STEP = 1

def handler(signum, frame):
    global going_on
    going_on = False
    print "receive a signal %d, going_on = %d"%(signum, going_on)

def updatePriusPos(pr):
    global thread_going
    while thread_going:
        pr.update_pos()

def updatePriusCollision(pr):
    global thread_going
    while thread_going:
        pr.update_collisions()

def enjoyPrius(args):
    # prius and its's environment
    prius = simulator.Prius(args)
    prius.reset()
    time.sleep(2)

def getAction(qv):
    action_array = np.zeros([ACTIONS])
    action_array[np.argmax(qv)] = 1
    action_angle = (np.argmax(qv) - 7) * 1.1
    return action_array, round(action_angle,2)

def getRandomAction(qv):
    random_value = random.randint(-7,7)
    random_angle = random_value * 1.1
    random_array = np.zeros([ACTIONS])
    random_array[random_value + 7] = 1
    return random_array, random_angle

def main():
    argparser = argparse.ArgumentParser(
        description='Enjoy Playing')
    argparser.add_argument(
        '--pose_address',
        metavar='PA',
        default='tcp://127.0.0.1:5563',
        help='Address of the pose (default: tcp://127.0.0.1:5563)')
    argparser.add_argument(
        '--pose_topic',
        metavar='PT',
        default='prius_pose',
        help='Topic of the pose (default: prius_pose)')
    argparser.add_argument(
        '--velocity_address',
        metavar='VA',
        default='tcp://127.0.0.1:5565',
        help='Address of the velocity (default: tcp://127.0.0.1:5565)')
    argparser.add_argument(
        '--velocity_topic',
        metavar='VT',
        default='prius_velocity',
        help='Topic of the velocity (default: prius_velocity)')
    argparser.add_argument(
        '--collision_address',
        metavar='CA',
        default='tcp://127.0.0.1:5566',
        help='Address of the collision (default: tcp://127.0.0.1:5566)')
    argparser.add_argument(
        '--collision_topic',
        metavar='CT',
        default='prius_collision',
        help='Topic of the collision (default: prius_collision)')
    argparser.add_argument(
        '--prius_control_address',
        metavar='PCA',
        default='tcp://127.0.0.1:5568',
        help='Address of the prius control (default: tcp://127.0.0.1:5568)')
    argparser.add_argument(
        '--prius_control_topic',
        metavar='PCT',
        default='prius_control',
        help='Topic of the prius control (default: prius_control)')
    argparser.add_argument(
        '--world_control_address',
        metavar='WCA',
        default='tcp://127.0.0.1:5569',
        help='Address of the world control (default: tcp://127.0.0.1:5569)')
    argparser.add_argument(
        '--world_control_topic',
        metavar='WCT',
        default='world_control',
        help='Topic of the world control (default: world_control)')
    argparser.add_argument(
        '--mode',
        default='0',
        type=int,
        help='0:control prius, 1:control world(default: 0)')
    argparser.add_argument(
        '--road',
        default='./map/road.box',
        help='road box file path')
    argparser.add_argument(
        '--vehicle',
        default='./map/vehicle.box',
        help='vehicle box file path')
    argparser.add_argument(
        '--track',
        default='./map/track.point',
        help='vehicle box file path')
    argparser.add_argument(
        '--value',
        default='0.3',
        type=float,
        help='prius gas pedel percent(default: 0)')

    args = argparser.parse_args()
    enjoyPrius(args)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
