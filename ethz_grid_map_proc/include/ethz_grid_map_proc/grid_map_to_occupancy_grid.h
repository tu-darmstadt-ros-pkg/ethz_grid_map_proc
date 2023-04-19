#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <hector_obstacle_msgs/ObstacleModel.h>

#include <dynamic_reconfigure/server.h>
#include <ethz_grid_map_proc/GridMapProcConfig.h>

class GridMapToOccupancyGrid
{
public:

  GridMapToOccupancyGrid(ros::NodeHandle nh, ros::NodeHandle pnh);
      
  void obstacleMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

  void obstacleObjectCallback(const hector_obstacle_msgs::ObstacleModelConstPtr msg);

  void gridMapCallback(const grid_map_msgs::GridMapConstPtr msg);
  
  void pathCallback(const nav_msgs::PathConstPtr msg);

  void sysCommandCallback(const std_msgs::StringConstPtr msg);
  
  void clear();
  
  void reconfigureCallback(ethz_grid_map_proc::GridMapProcConfig &config, uint32_t level);

private:
  void updateFusedMap();
  ros::Publisher occ_grid_pub_;
  ros::Publisher occ_grid_raw_pub_;
  ros::Publisher global_occ_grid_pub_;

  ros::Publisher grid_map_pub_;

  ros::Subscriber obstacle_grid_map_sub_;
  ros::Subscriber grid_map_sub_;
  ros::Subscriber obstacle_object_sub_;
  ros::Subscriber syscommand_sub_;
  ros::Subscriber path_sub_;
  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hector_obstacle_msgs::ObstacleModel obstacle_model_;
  grid_map::GridMap global_map_;
  
  typedef dynamic_reconfigure::Server<ethz_grid_map_proc::GridMapProcConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> dyn_rec_server_;
  boost::recursive_mutex config_mutex_;
  
  // Parameters
  std::string traversability_layer_name_;

  // Dynamic reconfigure parameters
  float p_occupied_threshold_;
  float p_goal_clear_radius_;
  bool p_obstacle_u_forward_;
  bool p_obstacle_u_backward_;
  float p_obstacle_u_size_;
  bool p_enable_obstacle_map;
  bool p_enable_traversability_map;
  bool p_unknown_space_to_free_;
  
  nav_msgs::Path path;
};
