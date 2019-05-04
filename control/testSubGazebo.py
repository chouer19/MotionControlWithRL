import zmq 
import sys 
import google.protobuf.text_format

import msg.pose_pb2 as GZ_pose

dest = "127.0.0.1"
port = 5563

context    = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5563")
subscriber.setsockopt(zmq.SUBSCRIBE, "demo")
c = GZ_pose.Pose()
while True:

    (topic, msg) = subscriber.recv_multipart()
    c.ParseFromString(msg)
    print "topic='%s' content='%s'" % (topic, str(c))

