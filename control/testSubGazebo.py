import zmq 
import sys 
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose
import msg.world_control_pb2 as GZ_world_control

world_control = GZ_world_control.WorldControl()
world_control.reset.all = True
print str(world_control)

dest = "127.0.0.1"
port = 5563

context    = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5563")
#subscriber.setsockopt(zmq.SUBSCRIBE, "prius_pose")
subscriber.setsockopt(zmq.SUBSCRIBE, "demo")
c = GZ_pose.Pose()
while True:

    (topic, msg) = subscriber.recv_multipart()
    c.ParseFromString(msg)
    print "topic='%s' content='%s'" % (topic, str(c))

