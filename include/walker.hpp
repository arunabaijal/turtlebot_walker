/*@
 *@file walker.hpp
 *@author Aruna Baijal
 *@brief This is the walker class.
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
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/* @brief walker class
 *
 * Class contains variable to read from sensor data and functions to facilitate
 * turning when required
 */
class Walker {
 private:
  bool obstacle;
  geometry_msgs::Twist msg;
  ros::NodeHandle nh;
  ros::Publisher pubVelocities;
  ros::Subscriber sub;
  float linearVel;
  float angularVel;

 public:
 /**
  * @brief constructor
  */
  Walker();
  /**
   * @brief destructor
   */
  ~Walker();
  /**
   * @brief  callback function to read visual data
   * @param  msg Pointer to messages from LaserScan
   * @return  none.
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   * @brief   Checks for obstacles nearby
   * @return  bool true for obstacle, false for clear path.
   */
  bool checkObstacle();
  /**
   * @brief   moves the turtlebot
   * @return  none.
   */
  void walk();
};

#endif  //  INCLUDE_WALKER_HPP_"