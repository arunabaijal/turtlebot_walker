#ifndef WALKER_HPP_
#define WALKER_HPP_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/* @brief
 * Define Roomba class
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
  Walker();
  ~Walker();
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  bool checkObstacle();
  void walk();
};

#endif  //  INCLUDE_WALKER_HPP_"
