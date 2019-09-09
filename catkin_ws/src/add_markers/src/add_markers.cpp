#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int64.h"

class AddMarkers
{
public:
  AddMarkers();

private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber pick_sub;
  int status;

  bool subscriber_exist;

  visualization_msgs::Marker marker;

  void pickCallback(const std_msgs::Int64 &msg);
  void update();
};

AddMarkers::AddMarkers() {
  status = 0;
  subscriber_exist = false;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  pick_sub = n.subscribe("/pick",1,&AddMarkers::pickCallback,this);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //marker.pose.position.x = 0.0;
  //marker.pose.position.y = 0.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}

void AddMarkers::pickCallback(const std_msgs::Int64 &msg) {
  status = msg.data;
  this->update();
}

void AddMarkers::update() {
    if (!subscriber_exist) {
      if (marker_pub.getNumSubscribers() > 0) {
        subscriber_exist = true;
      }
      else {
        if (!ros::ok())
        {
          return;
        }
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
    }

    switch (status) {
      case 1:
        ROS_INFO("add marker on pick up location.");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1.0;
        marker.pose.position.y = -1.5;
        break;
      case 2:
        ROS_INFO("picked up the object.");
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      case 3:
        ROS_INFO("dropped off the object.");
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -0.5;
        marker.pose.position.y = 0.5;
        break;
      default:
        break;
    }

    marker_pub.publish(marker);

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  AddMarkers addMarkers;
  ros::Rate r(1);
  ros::spin();

  return 0;
}
