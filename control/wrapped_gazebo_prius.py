import zmq 
import sys 
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose
import msg.contact_pb2 as GZ_contact
import msg.contacts_pb2 as GZ_contacts
import msg.world_control_pb2 as GZ_world_control
import msg.vector3d_pb2 as GZ_vector3d

class PriusSettings(object):
    def __init__(self, **kwargs):
        self.prius_pose_address = "tcp://127.0.0.1:5563"
        self.prius_pose_topic = "prius_pose"
        self.prius_velocity_address = "tcp://127.0.0.1:5565"
        self.prius_velocity_topic = "prius_velocity"
        self.prius_collision_address = "tcp://127.0.0.1:5566"
        self.prius_collision_topic = "prius_collision"
        self.prius_control_address = "tcp://127.0.0.1:5568"
        self.prius_control_topic = "prius_control"
        self.world_control_address = "tcp://127.0.0.1:5569"
        self.world_control_topic = "world_control"
        self.set(**kwargs)

    def set(self, **kwargs):
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise ValueError('PriusSettings: no key named %r' % key)
            setattr(self, key, value)
        pass

def make_prius_settings(args):
    settings = PriusSettings();
    settings.set(
            prius_pose_address = args.pose_address,
            prius_pose_topic = args.pose_topic,
            prius_velocity_address = args.velocity_address,
            prius_velocity_topic = args.velocity_topic,
            prius_collision_address = args.collision_address,
            prius_collision_topic = args.collision_topic,
            prius_control_address = args.prius_control_address,
            prius_control_topic = args.prius_control_topic,
            world_control_address = args.world_control_address,
            world_control_topic = args.world_control_topic
            )
    return settings
    pass

class Prius():
    def __init__(self,args):
        # prius settings
        self._settings = make_prius_settings(args)

        # context for interacting with network
        self._context = zmq.Context()

        # prius's pose subscriber
        self._pose_subscriber = self._context.socket(zmq.SUB)
        self._pose_subscriber.connect(self._settings.prius_pose_address)
        self._pose_subscriber.setsockopt(zmq.SUBSCRIBE, self._settings.prius_pose_topic)

        # prius's velocity subscriber
        self._velocity_subscriber = self._context.socket(zmq.SUB)
        self._velocity_subscriber.connect(self._settings.prius_velocity_address)
        self._velocity_subscriber.setsockopt(zmq.SUBSCRIBE, self._settings.prius_velocity_topic)

        # prius's collision subscriber
        self._collision_subscriber = self._context.socket(zmq.SUB)
        self._collision_subscriber.connect(self._settings.prius_collision_address)
        self._collision_subscriber.setsockopt(zmq.SUBSCRIBE, self._settings.prius_collision_topic)

        # prius's control publisher
        self._prius_control_publisher = self._context.socket(zmq.PUB)
        self._prius_control_publisher.bind(self._settings.prius_control_address)

        # world's control publisher
        self._world_control_publisher = self._context.socket(zmq.PUB)
        self._world_control_publisher.bind(self._settings.world_control_address)

        # msg for subscribing from simulator
        self._pose = GZ_pose.Pose()
        self._contacts = GZ_contacts.Contacts()
        self._speed = GZ_vector3d.Vector3d()

        # msg for publishing to simulator
        self._control = GZ_pose.Pose() # position.x() for gas pedal percent, and y() for steering wheel angle(radians from -7.85 to 7.85), z() for brake pedel percent.
        self._control.orientation.x = 0;
        self._control.orientation.y = 0;
        self._control.orientation.z = 0;
        self._control.orientation.w = 0;
        self._world_control = GZ_world_control.WorldControl()

    def pose(self):
        return self._pos()

    def _pos(self):
        topic = "aaa"
        while True:
            (topic, msg) = self._pose_subscriber.recv_multipart()
            if topic == self._settings.prius_pose_topic:
                self._pose.ParseFromString(msg)
                break
        return self._pose

    def velocity(self):
        return self._velocity()

    def _velocity(self):
        topic = "aaa"
        while True:
            (topic, msg) = self._velocity_subscriber.recv_multipart()
            print topic
            if topic == self._settings.prius_velocity_topic:
                self._speed.ParseFromString(msg)
                break
        return self._speed
    def collisions(self):
        return self._collisions()

    def _collisions(self):
        topic = "aaa"
        while True:
            (topic, msg) = self._collision_subscriber.recv_multipart()
            if topic == self._settings.prius_collision_topic:
                self._contacts.ParseFromString(msg)
                break
        return self._contacts

    def control_prius(self, gas_pedal = 0.3, hand_steering = 0, brake_pedal = 0):
        self._control_prius(gas_pedal, hand_steering, brake_pedal)

    def _control_prius(self, gas_pedal, hand_steering, brake_pedal):
        self._control.position.x = gas_pedal
        self._control.position.y = hand_steering
        self._control.position.z = brake_pedal
        buff = self._control.SerializeToString()
        self._prius_control_publisher.send_multipart([self._settings.prius_control_topic, buff])

    def control_world(self, reset = True):
        self._control_world(reset)

    def _control_world(self, reset):
        self._world_control.reset.all = reset 
        buff = self._world_control.SerializeToString()
        self._world_control_publisher.send_multipart([self._settings.world_control_topic, buff])

