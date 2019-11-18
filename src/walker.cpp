#include <iostream>
#include "walker.hpp"

Walker::Walker() {
  linearVel = 0.2;
  angularVel = 1.0;
  pubVelocities = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
      &Walker::laserCallback, this);
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVelocities.publish(msg);
}

Walker::~Walker() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVelocities.publish(msg);
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.8) {
      obstacle = true;
      return;
    }
  }
  obstacle = false;
}

bool Walker::checkObstacle() {
  return obstacle;
}

void Walker::walk() {
  ros::Rate loop(10);
  while (ros::ok()) {
    if (checkObstacle()) {
      ROS_INFO_STREAM("Obstacle present turning");
      msg.linear.x = 0.0;
      msg.angular.z = angularVel;
    } else {
      ROS_INFO_STREAM("Moving Forward");
      msg.angular.z = 0.0;
      msg.linear.x = linearVel;
    }
    
    pubVelocities.publish(msg);

    ros::spinOnce();
    loop.sleep();
  }
}
