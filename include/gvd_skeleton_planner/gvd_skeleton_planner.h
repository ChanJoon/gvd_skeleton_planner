#ifndef GVD_SKELETON_PLANNER_H
#define GVD_SKELETON_PLANNER_H

#include <ros/ros.h>
#include <string>

#include <voxblox_ros/esdf_server.h>
#include "voxblox_skeleton/ros/skeleton_vis.h"
#include "voxblox_skeleton/skeleton.h"
#include "voxblox_skeleton/skeleton_generator.h"
#include "voxblox_skeleton/skeleton_planner.h"
#include "voxblox_skeleton/sparse_graph_planner.h"

class GVDSkeletonPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GVDSkeletonPlanner(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  virtual ~GVDSkeletonPlanner() {}

  void generateSparseGraph();

  void run(const ros::TimerEvent& event);

  double getMapDistance(const Eigen::Vector3d& position) const;

 private:

  static constexpr int kMaxShortens = 10;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;

  ros::Timer gvd_timer_;

  std::string frame_id_;
  bool visualize_;
  double robot_radius_;
  double voxel_size_;  // Cache the size of the voxels used by the map.
  double esdf_max_distance_;
  float min_separation_angle_;
  int num_neighbors_for_edge_;
  float min_gvd_distance_;
  bool generate_by_layer_neighbors_;

  voxblox::EsdfServer voxblox_server_;
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;
  voxblox::SkeletonGenerator skeleton_generator_;

  // Planners of all sorts.
  voxblox::SkeletonAStar skeleton_planner_;
  voxblox::SparseGraphPlanner sparse_graph_planner_;
  
  bool graph_initialized_;

  bool shortenPath(const std::vector<Eigen::Vector3d>& path,
                   std::vector<Eigen::Vector3d>* shortened_path);
  bool shortenPathList(std::list<Eigen::Vector3d>::iterator start,
                       std::list<Eigen::Vector3d>::iterator end,
                       std::list<Eigen::Vector3d>* path_list);
  bool isLineInCollision(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

  bool getPathBetweenWaypoints(const Eigen::Vector3d& start,
                              const Eigen::Vector3d& goal,
                              std::vector<Eigen::Vector3d>* solution);
  void convertCoordinatePathToPath(
      const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
      std::vector<Eigen::Vector3d>* path) const;
};

#endif  // GVD_SKELETON_PLANNER_H
