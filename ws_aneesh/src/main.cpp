#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


std::string _name="Aneesh";
ros::Publisher player_out_pub;

/* 
ros::Publisher m_pub = n.advertise<std_msgs::Float32>("player_out", 1);
*/


void player_in_cb(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s heard: [%s]", _name.c_str(), msg->data.c_str());
    
  std_msgs::String msg_out;

  ROS_INFO("%s will publish a msg\n", _name.c_str());
  msg_out.data = "hello world";

  player_out_pub.publish(msg_out);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, _name);

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  player_out_pub = n.advertise<std_msgs::String>("player_out", 1);
  ros::Subscriber sub = n.subscribe("player_in", 1, player_in_cb);

  ros::Rate loop_rate(2);

  ROS_INFO("%s node started\n", _name.c_str());
 //int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    //++count;
  }

  return 0;
}

