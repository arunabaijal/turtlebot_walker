#include "walker.hpp"

/**
 * @brief main function
 * @param argc
 * @param argv
 * @return none
 */
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "turtlebot_walker");
  Walker walker;
  walker.walk();
  return 0;
}
