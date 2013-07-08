#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "ws_referee/custom.h"
#include "ws_referee/MovePlayerTo.h"

#include "ws_referee/randomize.h"

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


ros::NodeHandle *n;

int winning = 1;

std::string _name="aneesh";
std::string _to_police="paulo";

ros::Publisher player_out_pub;
ros::Publisher marker_pub;

static tf::TransformBroadcaster* br;
static tf::TransformListener* listener;

tf::Transform transform;

double _posx, _posy;

void run(double dist_in)
{
  tf::Transform tf_tmp;
  tf_tmp.setOrigin( tf::Vector3(dist_in, 0.0, 0.0) );

  double rotation = get_random_deg()*M_PI/180.;
  tf_tmp.setRotation( tf::Quaternion( 0, 0, rotation, 1) );

  br->sendTransform(tf::StampedTransform(tf_tmp, ros::Time::now(), 
                                   "tf_"+_name, "tf_tmp_"+_name));

  ros::Duration(0.1).sleep();

  tf::StampedTransform tf_1;
  try{
       listener->lookupTransform("world", "tf_tmp_"+_name,  
                                  ros::Time(0), tf_1);
     }
  catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
     }

  transform = tf_1;
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                          "world", "tf_"+_name));

  ROS_INFO("%s current position = %lf", _name.c_str(), _posx);
}

void publish_marker(double x, double y)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "tf_"+_name;
  marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.;
  marker.pose.position.y = 0.;
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

  //marker.pose.position.x = _posx-0.3;
  marker.pose.position.x = -0.3;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = "Aneesh";
  marker_pub.publish(marker);

}


void post(std::string winner)
{

  ws_referee::custom msg_out;
  msg_out.winner = winner;
  msg_out.sender = _name;
  msg_out.dist = get_random_num();
  ROS_INFO("%s will publish a msg\n", _name.c_str());

  if(_posx > 5)
  {
    msg_out.winner = _name;
    ROS_INFO("\n\n\t\t\033[92m %s WON \033[0m\n\n", _name.c_str());
  }

  player_out_pub.publish(msg_out);
}


void player_in_cb(const ws_referee::custom::ConstPtr& msg_in)
{

  ROS_INFO("%s Ordered to travel %lf dist", _name.c_str(), msg_in->dist);

  bool should_police = true;

  tf::StampedTransform tf_aux;
  try
  {
     listener->lookupTransform("world", "tf_"+_to_police,  
                                  ros::Time(0), tf_aux);
  }
  catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
    should_police = false;
  }

  if(should_police)
  {
      if(!is_in_field(tf_aux.getOrigin().x(), tf_aux.getOrigin().y()))
      {
          ros::ServiceClient punish_the_client = n->serviceClient<ws_referee::MovePlayerTo>("move_player"+_to_police);
          ws_referee::MovePlayerTo srv;
          srv.request.new_pos_x = -5.0;
          srv.request.new_pos_y = 0.0;
          ROS_INFO("aneesh: mike was out, calling service %s", "move_player_paulo");
          if (punish_the_client.call(srv))
          {
            ROS_INFO("Response to aneesh: %s", srv.response.reply.c_str());
          }
          {
            ROS_INFO("aneesh: service call failed. Now who will police the police?");
          }
      }
  }  

  run(msg_in->dist);

  publish_marker(_posx, _posy);

  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tf_"+_name));

  post(msg_in->winner);

  if(msg_in->winner != "")
  {
    ROS_INFO("\n\n\t\t\033[92m %s won :) \033[0m\n\n", msg_in->winner.c_str());
    winning = 0;
    return;
  }

}

bool srv_moveto_cb(ws_referee::MovePlayerTo::Request &req, 
                      ws_referee::MovePlayerTo::Response &res)
{

  //ROS_INFO("Aneesh ordered to move to (%lf, %lf). %s would you do this? you are a terrible person.", (float)req.new_pos_x + (float)req.new_pos_y, req.player_that_requested.c_str());
  ROS_INFO("Aneesh ordered to move to (%f, %f). %s why would you do this? you are a terrible person.", (float)req.new_pos_x, (float)req.new_pos_y, req.player_that_requested.c_str());

  transform.setOrigin( tf::Vector3(req.new_pos_x, req.new_pos_y, 0.0) );
  transform.setRotation( tf::Quaternion( 0, 0, 0, 1) );
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                           "world", "tf_"+_name));
  res.reply = "Aneesh SMASH!! "+ req.player_that_requested;
  ROS_INFO("%s", res.reply.c_str());
  return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, _name);

  n = (ros::NodeHandle *) new (ros::NodeHandle);

  //init the randomizer
  init_randomization_seed();

  //_posx = 0.;
  _posx = -5.;
  _posy = -2.78;


  player_out_pub = n->advertise<ws_referee::custom>("player_out", 1);
  marker_pub = n->advertise<visualization_msgs::Marker>("aneesh_marker", 1);

  ros::ServiceServer service = n->advertiseService("move_player_"+_name, srv_moveto_cb);

  br = (tf::TransformBroadcaster*) new (tf::TransformBroadcaster); 
  listener = (tf::TransformListener*) new (tf::TransformListener); 

  ros::Time t = ros::Time::now();

  //Send first transform 
  transform.setOrigin( tf::Vector3(_posx, _posy, 0.0) );
  transform.setRotation( tf::Quaternion( 0, 0, 0, 1) );
  br->sendTransform(tf::StampedTransform(transform, t,
                                           "world", "tf_"+_name));

  tf::Transform tf_tmp;
  tf_tmp.setOrigin( tf::Vector3(0, 0.0, 0.0) );
  tf_tmp.setRotation( tf::Quaternion( 0, 0, 0, 1) );

  br->sendTransform(tf::StampedTransform(tf_tmp, t,
                                           "tf_"+_name, "tf_tmp_"+_name));

  ros::spinOnce();
  ros::Duration(0.1).sleep();

  //Send first transform 
  transform.setOrigin( tf::Vector3(_posx, _posy, 0.0) );
  transform.setRotation( tf::Quaternion( 0, 0, 0, 1) );
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
                                           "world", "tf_"+_name));

  tf_tmp.setOrigin( tf::Vector3(0, 0.0, 0.0) );
  tf_tmp.setRotation( tf::Quaternion( 0, 0, 0, 1) );

  br->sendTransform(tf::StampedTransform(tf_tmp, ros::Time::now(), 
                                           "tf_"+_name, "tf_tmp_"+_name));
  ros::spinOnce();


  ros::Subscriber sub = n->subscribe("player_in", 1, player_in_cb);

  ros::Rate loop_rate(2);

  ROS_INFO("%s node started\n", _name.c_str());

  while (winning)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

