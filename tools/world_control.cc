#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include "zmq/zhelpers.hpp"

#include <ignition/transport/Node.hh>


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
    subscriber.connect("tcp://127.0.0.1:5569");
    subscriber.setsockopt( ZMQ_SUBSCRIBE, "world_control", 1);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::PublisherPtr worldControlPub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl msg;

    ignition::transport::Node ig_node;
    ignition::transport::Node::Publisher posePub = ig_node.Advertise<ignition::msgs::Any>("/prius/reset");

    while (true) {

        //  Read envelope with address
        std::string topic = s_recv (subscriber);
        std::cout << "[  topic ] :" << topic << std::endl;
        //  Read message contents
        std::string contents = s_recv (subscriber);
	msg.ParseFromString(contents.c_str());
	std::cout << msg.DebugString();
	worldControlPub->Publish(msg);
	posePub.Publish(ignition::msgs::Any());
    }
    return 0;

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
