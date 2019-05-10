import zmq 
import sys 
import argparse
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose
import msg.world_control_pb2 as GZ_world_control

import wrapped_gazebo_prius as simulator
import environment as En

import time
import random
import math

def enjoyPrius(args):
    #prius = simulator.Prius(args)
    env = En.Env(args.road, args.vehicle, args.track) 
    pos = GZ_pose.Pose()

    poses = []
    with open("./map/track.point") as f:
        line = f.readline()
        while line:
            contents = line.split('\t')
            line = f.readline()
            if len(contents) == 3:
                pp = GZ_pose.Pose()
                pp.position.x, pp.position.y, pp.orientation.x = float(contents[0]),float(contents[1]),float(contents[2]) + math.pi/2
                poses.append(pp)

    #for i in range(0,90):
    #for i in range(-90,96):
    #    pos.position.x = i * 0.5
    #    pos.position.y = random.random() * 2 - 1
    #    pos.position.y = 0
    #    pos.orientation.x = random.random()/20 - 0.025
    #    pos.orientation.x = 0
    for pp in poses:
        #pp.position.x = 45
        #pp.position.y = 0
        #pp.orientation.x = 0
        print(env.testRender(pp))
        #env.testRender(pos)
        time.sleep(0.05)
    #for i in range(0,90):
    #    pos.position.x = 42 + i/30
    #    pos.position.y = i / 30
    #    pos.orientation.x = i/90 * math.pi/2
    #    print str(pos)
    #    print(env.testRender(pos))
    #    #env.testRender(pos)
    #    time.sleep(0.05)

    #time.sleep(0.5)
    #while True:
    #    if args.mode == 0:
    #        prius.control_prius(args.value,-prius.pose().position.y/80,0)
    #    if args.mode == 1:
    #        prius.control_world()
    #        break
    #    time.sleep(0.1)
    #while True:
        #prius.control_world(False)
        #time.sleep(5)
        #print str(prius.pose())
        #if args.mode == 0:
        #    #prius.control_prius(args.value,-prius.pose().position.y/80,0)
        #    #prius.control_prius(args.value,-prius.pose().position.y/20,0)
        #    prius.control_prius(args.value,0.2,0)
        #if args.mode == 1:
        #    prius.control_world()
        #    break
        #print prius.pose().orientation.x
        #print str(prius.collisions())
        #print prius.collisions().position.x
        #print str(prius.velocity())
        print '===================='
    pass

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
    argparser.add_argument(
        '-m', '--map-name',
        metavar='M',
        default='Town01',
        help='plot the map of the current city (needs to match active map in '
             'server, options: Town01 or Town02)')

    args = argparser.parse_args()
    enjoyPrius(args)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
