import zmq 
import sys 
import argparse
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose
import msg.pose_pb2 as GZ_pose
import msg.world_control_pb2 as GZ_world_control
import msg.vector3d_pb2 as GZ_vector3d

import wrapped_gazebo_prius as simulator
import environment_avoid as En

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
INITIAL_EPSILON = 0.35 # starting value of epsilon
INITIAL_EPSILON = 0.1 # starting value of epsilon
OBSERVE = 80. # timesteps to observe before training
OBSERVE = 50000. # timesteps to observe before training
OBSERVE = 8000. # timesteps to observe before training
REPLAY_MEMORY = 50000 # number of previous transitions to remember
EXPLORE = 3000000. # frames over which to anneal epsilon
#EXPLORE = 2000000. # frames over which to anneal epsilon
FINAL_EPSILON = 0.05 # final value of epsilon
GAMMA = 0.98 # decay rate of past observations
BATCH = 32
INITIAL_STEP = 1

def handler(signum, frame):
    global going_on
    going_on = False
    print "receive a signal %d, going_on = %d"%(signum, going_on)

def updatePriusPos(pr):
    global going_on
    while going_on:
        pr.update_pos()

def updatePriusCollision(pr):
    global going_on
    while going_on:
        pr.update_collisions()

def updatePriusVelocity(pr):
    global going_on
    while going_on:
        pr.update_velocity()

def print_thread():
    global going_on
    while going_on:
        print("prnitprintprint")

def enjoyPrius(args):
    # controlling loop
    global going_on
    signal.signal(signal.SIGINT, handler)
    going_on = True

    env = En.Env(args.road, args.vehicle, args.track) 

    # network session and it's parameters
    sess = tf.InteractiveSession()
    inputState, outputQ, h_fc1 = createNetwork()
    #
    # action is choosed by policy pi
    #action = tf.placeholder("float", [None, ACTIONS])
    ## optimal q value of actions taken just now
    #target_q = tf.placeholder("float", [None])
    ## real q value when these actions selected and actuated
    #action_q = tf.reduce_sum(tf.multiply(outputQ , action), reduction_indices=1)
    ## target is q -> *q
    #cost = tf.reduce_mean(tf.square(target_q - action_q))
    #train_step = tf.train.AdamOptimizer(1e-6).minimize(cost)

    # saving and loading networks
    saver = tf.train.Saver()
    sess.run(tf.initialize_all_variables())
    checkpoint = tf.train.get_checkpoint_state("zero_networks_add")
    if checkpoint and checkpoint.model_checkpoint_path:
        saver.restore(sess, checkpoint.model_checkpoint_path)
        print("Successfully loaded:", checkpoint.model_checkpoint_path)
    else:
        print("Could not find old network weights")

    sessAvoid = tf.InteractiveSession()
    sessAvoid.run(tf.initialize_all_variables())
    inputStateAvoid, outputQAvoid, h_fc1Avoid = createNetwork()
    action = tf.placeholder("float", [None, ACTIONS])
    target_q = tf.placeholder("float", [None])
    action_q = tf.reduce_sum(tf.multiply(outputQ , action), reduction_indices=1)
    cost = tf.reduce_mean(tf.square(target_q - action_q))
    train_step = tf.train.AdamOptimizer(1e-6).minimize(cost)
    saverAvoid = tf.train.Saver()
    sessAvoid.run(tf.initialize_all_variables())
    checkpointAvoid = tf.train.get_checkpoint_state("avoid_networks_add")
    if checkpointAvoid and checkpointAvoid.model_checkpoint_path:
        saverAvoid.restore(sessAvoid, checkpointAvoid.model_checkpoint_path)
        print("Successfully loaded:", checkpointAvoid.model_checkpoint_path)
    else:
        print("Could not find old network weights")

    prius = simulator.Prius(args)
    thread_update_pos = threading.Thread(target = updatePriusPos, args = (prius,))
    thread_update_collisions = threading.Thread(target = updatePriusCollision, args = (prius,))
    thread_update_velocity = threading.Thread(target = updatePriusVelocity, args = (prius,))
    thread_update_pos.start()
    thread_update_collisions.start()
    thread_update_velocity.start()
    prius.reset()
    time.sleep(0.5)
    prius.control_world()
    x_t, reward, terminal, intersection, errorYaw= env.render(prius.collisions(), prius.pose(), prius.velocity())
    terminal = prius.terminal() or terminal
    if terminal:
        reward = -1
    while terminal and going_on:
        print(terminal)
        prius.reset()
        time.sleep(0.5)
        #prius.update_velocity()
        #prius.update_collisions()
        #prius.update_pos()
        x_t, reward, terminal, intersection, errorYaw= env.render(prius.collisions(), prius.pose(), prius.velocity())
        terminal = prius.terminal() or terminal
        if terminal:
            reward = -1
        #x_t, reward, terminal, intersection = env.render(prius.conllided(), prius.pose(), prius.velocity())
    # control prius by pedal percent 0.2, hand steering is 0, brake pedal is 0
    prius.control_prius(0.4, 0, 0)
    #prius.update_velocity()
    #prius.update_collisions()
    #prius.update_pos()
    x_t, reward, terminal, intersection, errorYaw= env.render(prius.collisions(), prius.pose(), prius.velocity())
    reward = round(reward,2)
    terminal = prius.terminal() or terminal
    if terminal:
        reward = -1
    #x_t, reward, terminal, intersection = env.render(prius.conllided(), prius.pose(), prius.velocity())
    x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
    ret , x_t = cv2.threshold(x_t,1,255,cv2.THRESH_BINARY)
    state_t = np.stack((x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t), axis=2)
    state_t1 = state_t

    epsilon = INITIAL_EPSILON 
    step = INITIAL_STEP

    store = deque()
    simTime = time.time()

    action_array_t = np.zeros([ACTIONS])
    action_angle_t = 0

    while going_on:
        # always run at high frequency
        #prius.update_velocity()
        #prius.update_collisions()
        #prius.update_pos()
        x_t, reward, terminal, intersection, errorYaw= env.render(prius.collisions(), prius.pose(), prius.velocity())
        reward = round(reward,2)
        terminal = prius.terminal() or terminal
        if terminal:
            reward = -1
        #if len(store) > BATCH:
        if step > OBSERVE + INITIAL_STEP:
            # sample a minibatch to train
            minibatch = random.sample(store, BATCH)
            # get the batch variables
            state_j_batch = [d[0] for d in minibatch]
            action_batch = [d[1] for d in minibatch]
            reward_batch = [d[2] for d in minibatch]
            state_j1_batch = [d[3] for d in minibatch]
            # qV_batch = []
            target_q_batch = []
            ##q_action1_batch = outputQ.eval(feed_dict = {inputState: state_j1_batch})
            q_action1_batch = outputQAvoid.eval(feed_dict = {inputStateAvoid: state_j1_batch})
            for i in range(0, len(minibatch)):
                terminal_j = minibatch[i][4]
                if terminal_j:
                    target_q_batch.append(reward_batch[i])
                else:
                    target_q_batch.append(reward_batch[i] + GAMMA * ( np.max(q_action1_batch[i])) )
            train_step.run(feed_dict = {
                target_q: target_q_batch,
                action: action_batch,
                inputState: state_j_batch }
                )
        # save progress every 10000 iterations
        #if step % 10000 == 0:
        if step % 10000 == 0:
            saverAvoid.save(sessAvoid, 'avoid_networks_add/' + GAME + '-dqn', global_step = step)
        # step > OBSERVE end train_step
 
        #prius.control_prius(0.3, -1*action_angle_t, 0)
        # control by frequecy of 10HZ
        if step < 50000 + INITIAL_STEP and (time.time() - simTime < 0.098) and (not terminal):
            continue
        simTime = time.time()

        ## current q value
        #readout_t = readout.eval(feed_dict={s : [s_t]})[0]
        qV_t = outputQ.eval(feed_dict={inputState : [state_t]})[0]
        qV_tAvoid = outputQAvoid.eval(feed_dict={inputStateAvoid : [state_t]})[0]
        ## epsilon-greedy policy
        action_array_t, action_angle_t = getAction(qV_t)
        action_array_tAvoid, action_angle_tAvoid = getAction(qV_tAvoid)
        #base_angle_t = math.sin(max(-1*math.pi * 0.5, min(math.pi * 0.5,errorYaw *2.0))) * 7.4
        base_angle_t = math.sin(max(-1*math.pi * 0.5, min(math.pi * 0.5,errorYaw *2.5))) * 7 + action_angle_t
        base_angle_t = round(base_angle_t,2)
        if step % 5 ==0 or terminal:
            printPink("   action from q value : "+str(action_angle_tAvoid) )
            print("base angle is : " + str(base_angle_t))
            print("sum angle is : " + str(base_angle_t + action_angle_tAvoid))
        # scale down epsilon
        if epsilon > FINAL_EPSILON and step > 50000 + INITIAL_STEP:
            epsilon -= (INITIAL_EPSILON - FINAL_EPSILON) / EXPLORE
        if random.random() < epsilon:
            action_array_t, action_angle_t = getRandomAction(qV_tAvoid)
            action_angle_t = round(action_angle_t,2)
            if step % 5 ==0 or terminal:
                printYellow("   action from random : "+str(action_angle_tAvoid) )
                print("base angle is : " + str(base_angle_t))
                print("sum angle is : " + str(base_angle_t + action_angle_t))

        prius.control_prius(0.6, base_angle_t + action_angle_tAvoid , 0)
        #prius.control_prius(0.6, base_angle_t , 0)
        x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
        ret , x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
        x_t = np.reshape(x_t, (160, 160, 1))
        state_t1 = np.append(x_t, state_t[:, :, :9], axis=2)
        store.append((state_t, action_array_t, reward, state_t1, terminal))
        if len(store) > REPLAY_MEMORY:
            store.popleft()
        # print on-time reward
        line1 = str(step) + " \tstep and reward is :" + str(reward)
        if step % 5 ==0 or terminal:
            print(round(base_angle_t,2),'\t')
            print(round(action_angle_t,2))
            if terminal:
                printRed(line1)
            elif reward < 0.15:
                printYellow(line1)
            elif reward < 0.3:
                printCyan(line1)
            else:
                printGreen(line1)
        # if terminal, restart
        if terminal:
            prius.reset()
            time.sleep(0.5)
            #prius.update_velocity()
            #prius.update_collisions()
            #prius.update_pos()
            x_t, reward, terminal, intersection, errorYaw= env.render(prius.collisions(), prius.pose(), prius.velocity())
            terminal = prius.terminal() or terminal
            if terminal:
                reward = -1
            #x_t, reward, terminal, intersection= env.render(prius.conllided(), prius.pose(), prius.velocity())
            x_t = cv2.cvtColor(cv2.resize(x_t, (160, 160)), cv2.COLOR_BGR2GRAY)
            ret , x_t = cv2.threshold(x_t, 1, 255, cv2.THRESH_BINARY)
            state_t1 = np.stack((x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t, x_t), axis=2)
        state_t = state_t1
        step += 1
    while going_on:
        time.sleep(2)
        print("dafdfweewq")
    prius.control_world()

def getAction(qv):
    action_array = np.zeros([ACTIONS])
    action_array[np.argmax(qv)] = 1
    action_angle = (np.argmax(qv) - 7) * 0.3
    return action_array, round(action_angle,2)

def getRandomAction(qv):
    random_value = random.randint(-7,7)
    print(random_value)
    action_angle = random_value * 0.3
    print(action_angle)
    random_array = np.zeros([ACTIONS])
    random_array[random_value + 7] = 1
    return random_array, action_angle

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
