/*@
 *@file walker.cpp
 *@author Aruna Baijal
 *@brief This is the walker/roomba class.
 *@copyright 2019 Aruna Baijal
 */
/**
 *BSD 3-Clause License
 *
 *Copyright 2019, Aruna
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *this list of conditions and the following disclaimer in the documentation
 *and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its contributors
 *may be used to endorse or promote products derived from this software without
 *specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THISSOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
#include <iostream>
#include "walker.hpp"

Walker::Walker() {
  linearVel = 0.2;
  angularVel = 1.0;
  pubVelocities = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
      &Walker::laserCallback, this);
  msg.linear.x = 0.0;  // set inital values
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  obstacle = false;
  pubVelocities.publish(msg);
}

Walker::~Walker() {
  msg.linear.x = 0.0;  // stop movement
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  pubVelocities.publish(msg);
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto i : msg->ranges) {
    if (i < 0.6) {  // check for obstacle distance
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
  ros::Rate loop(7);  // set ros loop rate
  while (ros::ok()) {
    if (checkObstacle()) {  // check for clear path
      ROS_WARN_STREAM("Obstacle detetced");
      msg.linear.x = 0.0;
      msg.angular.z = angularVel;  // turn
    } else {
      ROS_INFO_STREAM("No Obstacle");
      msg.angular.z = 0.0;
      msg.linear.x = linearVel;  // move forward
    }
    
    pubVelocities.publish(msg);

    ros::spinOnce();
    loop.sleep();
  }
}
