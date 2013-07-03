#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "ws_referee/custom.h"


std::string _name="Aneesh";
ros::Publisher player_out_pub;

double _posx, _posy;

void run(double dist_in)
{
  _posx += dist_in;
  ROS_INFO("%s current position = %lf", _name.c_str(), _posx);
}

void post()
{

  ws_referee::custom msg_out;
  msg_out.winner = "";
  msg_out.sender = _name;
  msg_out.dist = 0.5;
  ROS_INFO("%s will publish a msg\n", _name.c_str());

  player_out_pub.publish(msg_out);
}


void player_in_cb(const ws_referee::custom::ConstPtr& msg_in)
{
  ROS_INFO("%s Ordered to travel %lf dist", _name.c_str(), msg_in->dist);

  run(msg_in->dist);
  post();
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
  _posx = 0.;
  _posy = 6.;


  player_out_pub = n.advertise<ws_referee::custom>("player_out", 1);
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

