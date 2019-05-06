#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include "zmq/zhelpers.hpp"

#include <ignition/transport/Node.hh>
#include "ignition/msgs/vector3d.pb.h"
#include "ignition/msgs/quaternion.pb.h"
#include "ignition/msgs/pose.pb.h"


zmq::context_t context(1);
zmq::socket_t contacts_publisher(context, ZMQ_PUB);

/////////////////////////////////////////////////
// Function is called everytime a message is received.

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    //  Prepare our context and subscriber
    zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://127.0.0.1:5568");
    subscriber.setsockopt( ZMQ_SUBSCRIBE, "prius_control", 1);

    // Create our node for communication
    gazebo::msgs::Pose msg;
    ignition::msgs::Vector3d pos;
    ignition::msgs::Quaternion rot;
    ignition::msgs::Pose currentPosition;

    currentPosition.set_allocated_position(&pos);
    currentPosition.set_allocated_orientation(&rot);

    ignition::transport::Node ig_node;
    ignition::transport::Node::Publisher priusControlVelPub = ig_node.Advertise<ignition::msgs::Pose>("/cmd_vel");

    while (true) {

        //  Read envelope with address
        std::string topic = s_recv (subscriber);
        std::cout << "[  topic ] :" << topic << std::endl;
        //  Read message contents
        std::string contents = s_recv (subscriber);
	msg.ParseFromString(contents.c_str());
	std::cout << msg.DebugString();

        pos.set_x(msg.position().x());
        pos.set_y(msg.position().y());
        pos.set_z(msg.position().z());
        rot.set_x(msg.orientation().x());
        rot.set_y(msg.orientation().y());
        rot.set_z(msg.orientation().z());
        rot.set_w(msg.orientation().w());
	//priusControlVelPub.Publish(ignition::msgs::Convert(msg));
	priusControlVelPub.Publish(currentPosition);
    }
    return 0;

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
