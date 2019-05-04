// include zmq header
#include "zmq/zhelpers.hpp"

// include msgs
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/pose.pb.h>


int main(){
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    // init zmq node of publisher
    publisher.bind("tcp://127.0.0.1:5563");
    // publish using zmq node
    gazebo::msgs::Vector3d pos;
    gazebo::msgs::Quaternion rot;
    gazebo::msgs::Pose currentPosition;
    currentPosition.set_allocated_position(&pos);
    currentPosition.set_allocated_orientation(&rot);
    while(true){
    pos.set_x(8);
    pos.set_y(9);
    pos.set_z(6);

    rot.set_x(9.2);
    rot.set_y(2.2);
    rot.set_z(3.2);
    rot.set_w(5.2);


        std::string buff;
        currentPosition.SerializeToString(&buff);
        s_sendmore(publisher, "demo");
        s_send(publisher, buff);
        //s_send(publisher, "1234567890");
	sleep(1);
    }

    return 1;
}
