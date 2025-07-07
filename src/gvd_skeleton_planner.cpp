#include "gvd_skeleton_planner/gvd_skeleton_planner.h"

GVDSkeletonPlanner::GVDSkeletonPlanner(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("map"),
      visualize_(true),
      voxblox_server_(nh_, nh_private_),
      skeleton_generator_() {
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("robot_radius", robot_radius_, robot_radius_);
  nh_private_.param("esdf_max_distance", esdf_max_distance_, esdf_max_distance_);
  nh_private_.param("min_separation_angle", min_separation_angle_, min_separation_angle_);
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge_, num_neighbors_for_edge_);
  nh_private_.param("min_gvd_distance", min_gvd_distance_, min_gvd_distance_);
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors_,
                    generate_by_layer_neighbors_);

  graph_initialized_ = false;

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);

  gvd_timer_ = nh_private_.createTimer(
      ros::Duration(0.1), &GVDSkeletonPlanner::run, this);

  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);
}

void GVDSkeletonPlanner::run(const ros::TimerEvent& event) {
  if (!tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
    ROS_ERROR_THROTTLE(10, "TSDF map also empty! Check voxel size!");
    return;
  }

  if (graph_initialized_) {
    return;
  }

  ROS_WARN("Generating ESDF layer from TSDF.");
  ROS_WARN("Before Blocks: tsdf %zu esdf %zu",
           tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks(),
           esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks());

  voxblox_server_.disableIncrementalUpdate();
  if (voxblox_server_.getEsdfMapPtr()
                 ->getEsdfLayerPtr()
                 ->getNumberOfAllocatedBlocks() == 0) {
    const bool full_euclidean_distance = true;
    voxblox_server_.updateEsdfBatch(full_euclidean_distance);
  }

  ROS_WARN("After Blocks: tsdf %zu esdf %zu",
           tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks(),
           esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks());
  ROS_WARN("Finished generating ESDF layer from TSDF.");

  double voxel_size = esdf_map_->getEsdfLayerPtr()->voxel_size();
  ROS_WARN(
    "Voxel size: %f VPS: %zu",
    esdf_map_->getEsdfLayerPtr()->voxel_size(),
    esdf_map_->getEsdfLayerPtr()->voxels_per_side());

  // Also make a new skeleton layer and load it.
  // Make this as an unmanaged raw pointer, since we'll give it to skeleton
  // generator to own.
  voxblox::Layer<voxblox::SkeletonVoxel>* skeleton_layer =
      new voxblox::Layer<voxblox::SkeletonVoxel>(
          esdf_map_->getEsdfLayerPtr()->voxel_size(),
          esdf_map_->getEsdfLayerPtr()->voxels_per_side());
  
  voxblox_server_.setTraversabilityRadius(robot_radius_);
  voxblox_server_.setEsdfMaxDistance(esdf_max_distance_);

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
  }

  // Now set up the skeleton generator.
  skeleton_generator_.setEsdfLayer(
      esdf_map_->getEsdfLayerPtr());
  skeleton_generator_.setSkeletonLayer(skeleton_layer);
  skeleton_generator_.setMinSeparationAngle(min_separation_angle_);
  skeleton_generator_.setMinGvdDistance(min_gvd_distance_);
  skeleton_generator_.setGenerateByLayerNeighbors(generate_by_layer_neighbors_);
  skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge_);

  // Set up the A* planners.
  skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayer());
  skeleton_planner_.setEsdfLayer(
      esdf_map_->getEsdfLayerPtr());
  skeleton_planner_.setMinEsdfDistance(robot_radius_);
  skeleton_generator_.generateSkeleton();
  ROS_INFO("Finished generating skeleton.");

  ROS_WARN("Generating skeleton graph.");
  // Generate the sparse graph.
  generateSparseGraph();
}

void GVDSkeletonPlanner::generateSparseGraph() {

  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Generated skeleton graph.");

  if (visualize_) {
    voxblox::Pointcloud pointcloud;
    std::vector<float> distances;
    skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(
        &pointcloud, &distances);
    
    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    voxblox::pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    skeleton_pub_.publish(ptcloud_pcl);

    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    voxblox::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    ROS_WARN("Graph vertices size: %zu", marker_array.markers[0].points.size());
    ROS_WARN("Graph edges size: %zu", marker_array.markers[1].points.size());
    sparse_graph_pub_.publish(marker_array);
  }

  // Set up the graph planner.
  sparse_graph_planner_.setGraph(&skeleton_generator_.getSparseGraph());
  sparse_graph_planner_.setup();

  graph_initialized_ = true;
}

double GVDSkeletonPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!esdf_map_) {
    return 0.0;
  }
  double distance = 0.0;
  if (!esdf_map_->getDistanceAtPosition(position, &distance)) {
    ROS_ERROR_STREAM("Failed to get distance at position: " << position.transpose());
    return 0.0;
  }
  return distance;
}


bool GVDSkeletonPlanner::shortenPath(
  const std::vector<Eigen::Vector3d> &path,
  std::vector<Eigen::Vector3d> *shortened_path)
{
  std::list<Eigen::Vector3d> path_list(path.begin(), path.end());

  // Use the class-level constant for maximum shortens
  bool any_success = false;

  for (int i = 0; i < kMaxShortens; ++i)
  {
      if (!shortenPathList(path_list.begin(), path_list.end(), &path_list))
      {
          break;
      }
      any_success = true;
  }

  shortened_path->assign(path_list.begin(), path_list.end());
  return any_success;
}

bool GVDSkeletonPlanner::shortenPathList(
  std::list<Eigen::Vector3d>::iterator start,
  std::list<Eigen::Vector3d>::iterator end,
  std::list<Eigen::Vector3d> *path_list)
{
  if (start == end)
  {
      return false;
  }
  std::list<Eigen::Vector3d>::iterator start_iter = start;
  std::list<Eigen::Vector3d>::iterator last_iter = end;
  std::advance(last_iter, -1);

  if (!isLineInCollision(*start_iter, *last_iter))
  {
      std::advance(start_iter, 1);
      int i = 0;
      while (start_iter != last_iter && start_iter != end)
      {
          start_iter = path_list->erase(start_iter);
          ++i;
      }
      return i > 0;
  }
  else
  {
      std::list<Eigen::Vector3d>::iterator middle_iter = start;
      for (int i = 0; start_iter != last_iter; ++start_iter, ++i)
      {
          if (i % 2 == 0)
          {
              middle_iter++;
          }
      }

      if (middle_iter == start_iter || middle_iter == end)
      {
          return false;
      }

      bool left_success = shortenPathList(start, middle_iter, path_list);
      bool right_success = shortenPathList(middle_iter, end, path_list);
      return left_success || right_success;
  }
  return false;
}

bool GVDSkeletonPlanner::isLineInCollision(
  const Eigen::Vector3d &start,
  const Eigen::Vector3d &end)
{
  Eigen::Vector3d direction = end - start;
  double distance = direction.norm();
  direction.normalize();

  if (distance < voxel_size_)
  {
      return false;
  }

  Eigen::Vector3d current = start;
  double total_distance = 0.0;

  while (total_distance <= distance)
  {
      voxblox::EsdfVoxel *esdf_voxel = esdf_map_->getEsdfLayerPtr()->getVoxelPtrByCoordinates(current.cast<voxblox::FloatingPoint>());
      if (esdf_voxel == nullptr)
      {
          return true;
      }
      if (esdf_voxel->distance < robot_radius_)
      {
          return true;
      }

      double step_size = std::max(voxel_size_, esdf_voxel->distance - robot_radius_);
      current += step_size * direction;
      total_distance += step_size;
  }
  return false;
}

// Fixed start and end locations, returns list of waypoints between.
bool GVDSkeletonPlanner::getPathBetweenWaypoints(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal,
  std::vector<Eigen::Vector3d>* solution) {
  voxblox::Point start_point = start.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point = goal.cast<voxblox::FloatingPoint>();

  voxblox::AlignedVector<voxblox::Point> graph_coordinate_path;
  bool success = sparse_graph_planner_.getPath(start_point, goal_point,
                                              &graph_coordinate_path);
  std::vector<Eigen::Vector3d> graph_path;
  convertCoordinatePathToPath(graph_coordinate_path, &graph_path);
  ROS_INFO("Got sparse graph path.");
  if (!success) {
    return false;
  }

  voxblox::AlignedVector<voxblox::Point> exact_start_path, exact_goal_path;

  success &= skeleton_planner_.getPathInEsdf(
      start_point, graph_coordinate_path.front(), &exact_start_path);
  success &= skeleton_planner_.getPathInEsdf(graph_coordinate_path.back(),
                                            goal_point, &exact_goal_path);

  graph_coordinate_path.insert(graph_coordinate_path.begin(),
                              exact_start_path.begin(),
                              exact_start_path.end());
  graph_coordinate_path.insert(graph_coordinate_path.end(),
                              exact_goal_path.begin(), exact_goal_path.end());
  convertCoordinatePathToPath(graph_coordinate_path, &graph_path);
  ROS_INFO("Got ESDF path.");


  *solution = graph_path;
  return success;
}

void GVDSkeletonPlanner::convertCoordinatePathToPath(
  const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
  std::vector<Eigen::Vector3d>* path) const {
  CHECK_NOTNULL(path);
  path->clear();
  path->reserve(coordinate_path.size());

  for (const voxblox::Point& voxblox_point : coordinate_path) {
    path->push_back(voxblox_point.cast<double>());
  }
}