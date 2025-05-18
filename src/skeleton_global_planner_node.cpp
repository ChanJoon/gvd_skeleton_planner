#include "gvd_skeleton/skeleton_global_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "skeleton_global_planner");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::SkeletonGlobalPlanner planner_node(nh, nh_private);
  ROS_INFO("Initialized skeleton global planner node.");

  ros::spin();
  return 0;
}
