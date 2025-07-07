#include "gvd_skeleton_planner/gvd_skeleton_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gvd_skeleton_planner");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  GVDSkeletonPlanner gvd_skeleton_planner(nh, nh_private);
  ROS_INFO("Initialized GVD skeleton planner node.");

  ros::spin();
  return 0;
}
