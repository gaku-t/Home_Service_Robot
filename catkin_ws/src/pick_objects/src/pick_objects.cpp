#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int64.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  // Goal Publisher
  ros::Publisher pub = n.advertise<std_msgs::Int64>("/pick",10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = -1.5;
  goal.target_pose.pose.orientation.w = 1.0;

  while (pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN("Waiting for add_markers to subscribe.");
    sleep(1);
  }

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for pickup location");
  ac.sendGoal(goal);
  std_msgs::Int64 msg;
  msg.data = 1;
  pub.publish(msg);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //ROS_INFO("Hooray, the base moved 1 meter forward");
    ROS_INFO("Hooray, the robot reached to the pickup location, 5 seconds to go");
    ros::Duration(5.0).sleep();
    msg.data = 2;
    pub.publish(msg);
  }
  else {
    ROS_INFO("The robot failed to reach to the pickup location, exiting");
    ros::Duration(5.0).sleep();
    return 0;
  }

  move_base_msgs::MoveBaseGoal drop_goal;

  // set up the frame parameters
  drop_goal.target_pose.header.frame_id = "map";
  drop_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position for the robot to reach
  drop_goal.target_pose.pose.position.x = -0.5;
  drop_goal.target_pose.pose.position.y = 0.5;
  drop_goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for drop off location");
  ac.sendGoal(drop_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    msg.data = 3;
    pub.publish(msg);
    ROS_INFO("SUCCESS, the robot reached to the drop off location, exiting");
  }
  else {
    ROS_INFO("The robot failed to reach to the drop off location, exiting");
  }
  ros::Duration(5.0).sleep();
  return 0;
}
