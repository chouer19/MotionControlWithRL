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

def handler(signum, frame):
    global going_on
    going_on = False
    print "receive a signal %d, going_on = %d"%(signum, going_on)

# param definition
GAME = 'angry-car' # the name of the game being played for log files
ACTIONS = 20 # number of valid actions
INITIAL_EPSILON = 0.0001 # starting value of epsilon
INITIAL_EPSILON = 0.16 # starting value of epsilon
#INITIAL_EPSILON = 0.2 # starting value of epsilon
OBSERVE = 100000. # timesteps to observe before training
EXPLORE = 3000000. # frames over which to anneal epsilon
#EXPLORE = 2000000. # frames over which to anneal epsilon
FINAL_EPSILON = 0.0001 # final value of epsilon
FINAL_EPSILON = 0.0 # final value of epsilon
GAMMA = 0.9 # decay rate of past observations
BATCH = 32

def updatePriusPos(pr):
    global going_on
    while going_on:
        pr.update_pos()

def updatePriusCollision(pr):
    global going_on
    while going_on:
        pr.update_collisions()

def enjoyPrius(args):
    # controlling loop
    global going_on
    signal.signal(signal.SIGINT, handler)
    going_on = True

    # prius and its's environment
    prius = simulator.Prius(args)
    thread_update_pos = threading.Thread(target = updatePriusPos, args = (prius,))
    thread_update_collisions = threading.Thread(target = updatePriusCollision, args = (prius,))
    thread_update_pos.start()
    thread_update_collisions.start()
    env = En.Env(args.road, args.vehicle, args.track) 

    # network session and it's parameters
    sess = tf.InteractiveSession()
    inputState, outputQ, h_fc1 = createNetwork()
    
    # action is choosed by policy pi
    action = tf.placeholder("float", [None, ACTIONS])
    # optimal q value of actions taken just now
    target_q = tf.placeholder("float", [None])
    # real q value when these actions selected and actuated
    action_q = tf.reduce_sum(tf.multiply(outputQ , action), reduction_indices=1)
    # target is q -> *q
    cost = tf.reduce_mean(tf.square(target_q - action_q))
    train_step = tf.train.AdamOptimizer(1e-6).minimize(cost)

    # saving and loading networks
    saver = tf.train.Saver()
    sess.run(tf.initialize_all_variables())
    checkpoint = tf.train.get_checkpoint_state("saved_networks")
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)
        print("Successfully loaded:", checkpoint.model_checkpoint_path)
    else:
        print("Could not find old network weights")

    # control prius by pedal percent 0.2, hand steering is 0, brake pedal is 0
    prius.control_prius(0.2, 0, 0)
    x_t, reward, terminal = env.render(prius.collisions(), prius.pose())
    x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
    ret , x_t = cv2.threshold(x_t,1,255,cv2.THRESH_BINARY)
    state_t = np.stack((x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t), axis=2)
    state_t1 = state_t

    epsilon = INITIAL_EPSILON
    step = 1

    store = deque()
    simTime = time.time()

    while going_on:
        # always run at high frequency
        x_t, reward, terminal = env.render(prius.collisions(), prius.pose())
        # train when step over observation stage
        if step > OBSERVE:
            # sample a minibatch to train
            minibatch = random.sample(store, BATCH)
            # get the batch variables
            state_j_batch = [d[0] for d in minibatch]
            action_batch = [d[1] for d in minibatch]
            reward_batch = [d[2] for d in minibatch]
            state_j1_batch = [d[3] for d in minibatch]
            # qV_batch = []
            target_q_batch = []
            q_action1_batch = outputQ.eval(feed_dict = {inputState: state_j1_batch})
            for i in range(0, len(minibatch)):
                terminal = minibatch[i][4]
                if terminal:
                    target_q_batch.append(reward_batch)
                else:
                    target_q_batch.append(reward_batch[i] + GAMMA * \
                                      ( np.max(q_action1_batch[i][0:2] ) + \
                                        np.max(q_action1_batch[i][2:4] ) + \
                                        np.max(q_action1_batch[i][4:6] ) + \
                                        np.max(q_action1_batch[i][6:8] ) + \
                                        np.max(q_action1_batch[i][8:10] ) + \
                                        np.max(q_action1_batch[i][10:12]) +\
                                        np.max(q_action1_batch[i][12:14]) +\
                                        np.max(q_action1_batch[i][14:16]) +\
                                        np.max(q_action1_batch[i][16:18]) +\
                                        np.max(q_action1_batch[i][18:20])
                                     )  )
            train_step.run(feed_dict = {inputState: state_j_batch, target_q: target_q_batch, action:action_batch })
        # step > OBSERVE end train_step
        
        # save progress every 10000 iterations
        if step % 10000 == 0:
            saver.save(sess, 'saved_networks/' + GAME + '-dqn', global_step = step)

        line1 = "step :====================== ",step,"======================== "
        line3 = "reward                       ",reward,"                      "

        # control by frequecy of 10HZ
        if (time.time() - simTime < 0.98) and (not terminal):
            continue
        simTime = time.time()

        ## current q value
        #readout_t = readout.eval(feed_dict={s : [s_t]})[0]
        qV_t = outputQ.eval(feed_dict={inputState : [state_t]})[0]
        ## epsilon-greedy policy
        action_array_t = np.zeros([ACTIONS])
        action_angle_t = 0
        if random.random() < epsilon:
            action_array_t, action_angle_t = getRandomAction()
        else:
            action_array_t, action_angle_t = getAction(qV_t)
        # scale down epsilon
        if epsilon > FINAL_EPSILON and step > OBSERVE:
            epsilon -= (INITIAL_EPSILON - FINAL_EPSILON) / EXPLORE

        prius.control_prius(0.2, -1*action_angle_t, 0)
        x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
        ret , x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
        x_t = np.reshape(x_t, (160, 160, 1))
        state_t1 = np.append(x_t, state_t[:, :, :9], axis=2)

        store.append((state_t, action_array_t, reward, state_t1, terminal))
        if terminal:
            prius.reset()
            time.sleep(0.2)
            x_t, reward, terminal = env.render(prius.collisions(), prius.pose())
            x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
            ret , x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
            state_t1 = np.stack((x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t), axis=2)
            printRed(line1)
            printRed(line3)
        elif reward < 0.04:
            printYellow(line1)
            printYellow(line3)
        elif reward < 0.1:
            printCyan(line1)
            printCyan(line3)
        else:
            printGreen(line1)
            printGreen(line3)

        state_t = state_t1
        step += 1

def getAction(qv):
    action_array = np.zeros([ACTIONS])
    direction_index = np.argmax(qv[0:2])
    angle1_index = np.argmax(qv[2:4]) + 2
    angle2_index = np.argmax(qv[4:6]) + 4
    angle4_index = np.argmax(qv[6:8]) + 6
    angle8_index = np.argmax(qv[8:10]) + 8
    angle16_index = np.argmax(qv[10:12]) + 10
    angle32_index = np.argmax(qv[12:14]) + 12
    angle64_index = np.argmax(qv[14:16]) + 14
    angle128_index = np.argmax(qv[16:18]) + 16
    angle256_index = np.argmax(qv[18:20]) + 18

    action_angle = 0.015 * (-1 * qv[0] + qv[1]) * \
                                      (qv[3] + qv[5]*2 + qv[7]*4 + qv[9] * 8 + qv[11]*16 + \
                                       qv[13]*32 + qv[15]*64 + qv[17]*128 + qv[19]*256 )
    return action_array, action_angle

def getRandomAction():
    #random_value = int(max(-511, min(511, int(random.gauss(0,150))) ))
    random_value = int(random.uniform(-510,510))
    random_angle = random_value * 0.015
    random_array = np.zeros([ACTIONS])
    direction_index = 0
    if random_value < 0:
        direction_index = 0
    else:
        direction_index = 1
    random_value = abs(random_value)

    random_array[direction_index] = 1
    random_array[2 + random_value % 2] = 1
    random_value /= 2
    random_array[4 + random_value % 2] = 1
    random_value /= 2
    random_array[6 + random_value % 2] = 1
    random_value /= 2
    random_array[8 + random_value % 2] = 1
    random_value /= 2
    random_array[10 + random_value % 2] = 1
    random_value /= 2
    random_array[12 + random_value % 2] = 1
    random_value /= 2
    random_array[14 + random_value % 2] = 1
    random_value /= 2
    random_array[16 + random_value % 2] = 1
    random_value /= 2
    random_array[18 + random_value % 2] = 1

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
