#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "ws_referee/custom.h"
#include <visualization_msgs/Marker.h>


std::string _name="Aneesh";
ros::Publisher player_out_pub;
ros::Publisher marker_pub;

double _posx, _posy;

void run(double dist_in)
{
  _posx += dist_in;
  ROS_INFO("%s current position = %lf", _name.c_str(), _posx);
}

void publish_marker(double x, double y)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 0.25;
  marker.color.g = 0.40;
  marker.color.b = 0.7;

  marker_pub.publish(marker);

  //Add test based marker
  marker.id = _posy;
  marker.color.a = 1.0;
  marker.color.r = 0.25;
  marker.color.g = 0.40;
  marker.color.b = 0.9;

  marker.pose.position.x = _posx-0.3;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = "Aneesh";
  marker_pub.publish(marker);

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
  publish_marker(_posx, _posy);

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
  marker_pub = n.advertise<visualization_msgs::Marker>("aneesh_marker", 1);

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

