#include "path_follow/pure_pursuit.h"


/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "pure_pursuit_node");

  pure_pursuit test; // 规划测试

  return 0;
}

