#include <ethz_grid_map_proc/grid_map_to_occupancy_grid.h>
#include <ethz_grid_map_proc/GridMapProcConfig.h>

GridMapToOccupancyGrid::GridMapToOccupancyGrid(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh) ,pnh_(pnh)
{
  // Load parameters
  pnh_.param<std::string>("traversability_layer_name", traversability_layer_name_, "traversability");
  pnh_.param<std::string>("elevation_layer_name", elevation_layer_name_, "elevation");
  // Initialize dynamic reconfigure
  dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
  dyn_rec_server_->setCallback(boost::bind(&GridMapToOccupancyGrid::reconfigureCallback, this, _1, _2));

  // Initialize global map
  global_map_.add("obstacle");
  global_map_.add(traversability_layer_name_);
  global_map_.add("obstacle_objects");
  global_map_.add(elevation_layer_name_);
  global_map_.add("fused");
  global_map_.setGeometry(grid_map::Length(20.0, 20.0), 0.05);
  global_map_.setFrameId("world");
  this->clear();

  // Publishers
  occ_grid_raw_pub_    = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map_raw", 1);
  occ_grid_pub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map", 1);
  global_occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/global_map_debug", 1);

  // Subscribers
  obstacle_grid_map_sub_ = nh_.subscribe("/obstacle_map_throttled", 1, &GridMapToOccupancyGrid::obstacleMapCallback, this);
  obstacle_object_sub_ = nh_.subscribe("/hector_obstacle_server/obstacle_model", 1, &GridMapToOccupancyGrid::obstacleObjectCallback, this);

  grid_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &GridMapToOccupancyGrid::gridMapCallback, this);
  syscommand_sub_ = nh_.subscribe("/syscommand", 1, &GridMapToOccupancyGrid::sysCommandCallback, this);
  path_sub_ = nh_.subscribe("/path_to_follow", 1, &GridMapToOccupancyGrid::pathCallback, this);
}

void GridMapToOccupancyGrid::obstacleMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  grid_map::GridMap local_obstacle_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "obstacle", local_obstacle_map);
  std::vector<std::string> layers_to_use;
  layers_to_use.emplace_back("obstacle");
  global_map_.addDataFrom(local_obstacle_map, true, true, false, layers_to_use);

  if (!p_enable_traversability_map) {
    updateFusedMap();
  }
}

void GridMapToOccupancyGrid::obstacleObjectCallback(const hector_obstacle_msgs::ObstacleModelConstPtr msg)
{
  obstacle_model_ = *msg;
}

void GridMapToOccupancyGrid::gridMapCallback(const grid_map_msgs::GridMapConstPtr msg)
{
  // Convert message to grid map object
  grid_map::GridMap local_grid_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, local_grid_map);

  // Publish as occupancy map before tresholding
  if (occ_grid_raw_pub_.getNumSubscribers() > 0){
    nav_msgs::OccupancyGrid local_occupancy_grid;

    grid_map::GridMapRosConverter::toOccupancyGrid(local_grid_map, traversability_layer_name_, 1.0, 0.0, local_occupancy_grid);
    occ_grid_raw_pub_.publish(local_occupancy_grid);
  }

  // Threshold map and convert traversability to obstacle
  grid_map::Matrix& data = local_grid_map[traversability_layer_name_];
  for (grid_map::GridMapIterator iterator(local_grid_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    float& value = data(index(0), index(1));
    if (std::isnan(value)) {
      continue;
    }
    if (value < p_occupied_threshold_) {
      value = 100;
    } else {
      value = 0;
    }
  }

  // Insert current local traversability map into global map
  std::vector<std::string> layers_to_use;
  layers_to_use.push_back(traversability_layer_name_);
  layers_to_use.emplace_back(elevation_layer_name_);
  global_map_.addDataFrom(local_grid_map, true, true, false, layers_to_use);

  grid_map::Matrix& traversability_data = global_map_[traversability_layer_name_];

  // Clear waypoints
  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    grid_map::Position pos(path.poses[i].pose.position.x, path.poses[i].pose.position.y);

    for (grid_map::CircleIterator iterator(global_map_, pos, p_goal_clear_radius_);
           !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      traversability_data(index(0), index(1)) = 0.0;
    }

  }

  // Create obstacle U so robot drives forward/backward
  if (p_obstacle_u_forward_ || p_obstacle_u_backward_) {

        grid_map::Position left_top    (p_obstacle_u_size_, p_obstacle_u_size_);
        grid_map::Position left_bottom(-p_obstacle_u_size_, p_obstacle_u_size_);

        for (grid_map::LineIterator iterator(global_map_,  left_top, left_bottom);
            !iterator.isPastEnd(); ++iterator) {
          const grid_map::Index index(*iterator);
          traversability_data(index(0), index(1)) = 100.0;
        }

        grid_map::Position right_top    (p_obstacle_u_size_, -p_obstacle_u_size_);
        grid_map::Position right_bottom(-p_obstacle_u_size_, -p_obstacle_u_size_);

        for (grid_map::LineIterator iterator(global_map_, right_top, right_bottom);
            !iterator.isPastEnd(); ++iterator) {
          const grid_map::Index index(*iterator);
          traversability_data(index(0), index(1)) = 100.0;
        }

        if (p_obstacle_u_forward_) {
            for (grid_map::LineIterator iterator(global_map_, left_top, right_top);
                !iterator.isPastEnd(); ++iterator) {
              const grid_map::Index index(*iterator);
              traversability_data(index(0), index(1)) = 100.0;
            }
        }

        if (p_obstacle_u_backward_) {
            for (grid_map::LineIterator iterator(global_map_, left_bottom, right_bottom);
                !iterator.isPastEnd(); ++iterator) {
              const grid_map::Index index(*iterator);
              traversability_data(index(0), index(1)) = 100.0;
            }
        }

  }

  if (p_enable_traversability_map) {
    updateFusedMap();
  }

//  if (grid_map_pub_.getNumSubscribers() >0 )
//  {
//    grid_map_msgs::GridMap grid_map_msg;
//    grid_map::GridMapRosConverter::toMessage(global_map_, grid_map_msg);
//    grid_map_pub_.publish(grid_map_msg);
//  }


  if (occ_grid_pub_.getNumSubscribers() > 0){
    nav_msgs::OccupancyGrid local_occupancy_grid;

    grid_map::GridMapRosConverter::toOccupancyGrid(local_grid_map, traversability_layer_name_, 0.0, 100.0, local_occupancy_grid);
    occ_grid_pub_.publish(local_occupancy_grid);
  }

//    if (global_occ_grid_pub_.getNumSubscribers() > 0){

//      nav_msgs::OccupancyGrid occupancy_grid;

//      grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, traversability_layer_name_, 1.0, 0.0, occupancy_grid);
//      global_occ_grid_pub_.publish(occupancy_grid);
//    }
}

void GridMapToOccupancyGrid::pathCallback(const nav_msgs::PathConstPtr msg)
{
  path = *msg;
}

void GridMapToOccupancyGrid::sysCommandCallback(const std_msgs::StringConstPtr msg)
{
  if (msg->data == "reset" || msg->data == "reset_2d_map"){
    this->clear();
  }
}

void GridMapToOccupancyGrid::clear()
{
  global_map_.clearAll();
}

void GridMapToOccupancyGrid::reconfigureCallback(ethz_grid_map_proc::GridMapProcConfig &config, uint32_t level)
{
  p_occupied_threshold_ = config.occupied_threshold;
  p_goal_clear_radius_ = config.goal_clear_radius;
  p_obstacle_u_forward_ = config.obstacle_u_forward;
  p_obstacle_u_backward_ = config.obstacle_u_backward;
  p_obstacle_u_size_ = config.obstacle_u_size;
  p_enable_obstacle_map = config.enable_obstacle_map;
  p_enable_traversability_map = config.enable_traversability_map;
  p_unknown_space_to_free_ = config.unknown_space_to_free;

  ROS_INFO("Reconfigure Request. Occupied threshold: %f clear radius:%f, of: %s ,ob: %s, obstacle_map: %s, traversability_map: %s, unknown_space_to_free: %s",
           static_cast<double>(p_occupied_threshold_),
           static_cast<double>(p_goal_clear_radius_),
           config.obstacle_u_forward?"True":"False",
           config.obstacle_u_backward?"True":"False",
           config.enable_obstacle_map?"True":"False",
           config.enable_traversability_map?"True":"False",
           config.unknown_space_to_free?"True":"False");
}

void GridMapToOccupancyGrid::updateFusedMap()
{
  if (p_enable_obstacle_map && !p_enable_traversability_map) {
    // Only obstacle map
    global_map_["fused"] = global_map_["obstacle"];
  } else if (p_enable_traversability_map && !p_enable_obstacle_map) {
    // Only traversability map
    global_map_["fused"] = global_map_[traversability_layer_name_];
  } else if (!p_enable_traversability_map /*&& !p_enable_obstacle_map*/) {
    // Both maps disabled, add empty map
    global_map_.add("fused", NAN);
  } else {
    // Fuse both
    grid_map::Matrix& obstacle_data   = global_map_["obstacle"];
    grid_map::Matrix& traversability_data = global_map_[traversability_layer_name_];
    grid_map::Matrix& fused_data = global_map_["fused"];
    for (grid_map::GridMapIterator iterator(global_map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);

      if (std::isnan(obstacle_data(index(0), index(1)))) {
        fused_data(index(0), index(1)) = traversability_data(index(0), index(1));
      } else if (std::isnan(traversability_data(index(0), index(1))) ||
               obstacle_data(index(0), index(1)) >= traversability_data(index(0), index(1))) {
        fused_data(index(0), index(1)) = obstacle_data(index(0), index(1));
      } else {
        fused_data(index(0), index(1)) = traversability_data(index(0), index(1));
      }
    }
  }

  // add obstacle model
  if(!obstacle_model_.model.empty())
  {
    for( const hector_obstacle_msgs::Obstacle& obstacle : obstacle_model_.model)
    {
      if(obstacle.shape_type == hector_obstacle_msgs::Obstacle::SHAPE_LINE_WITH_ENDPOINTS)
      {
        if(obstacle.points.size() == 2)
        {
          grid_map::Position start(obstacle.points[0].x, obstacle.points[0].y);
          grid_map::Position end(obstacle.points[1].x, obstacle.points[1].y);

          for (grid_map::LineIterator iterator(global_map_, start, end); !iterator.isPastEnd(); ++iterator) {
            global_map_.at("fused", *iterator) = 100.0;
          }
        }
      }
    }
  }

  // Set all unknown space to free
  if (p_unknown_space_to_free_) {
    grid_map::Matrix& fused_data = global_map_["fused"];
    for (grid_map::GridMapIterator iterator(global_map_); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      if (std::isnan(fused_data(index(0), index(1)))) {
        fused_data(index(0), index(1)) = 0;
      }
    }
  }

  // publish fused map
  if (global_occ_grid_pub_.getNumSubscribers() > 0) {
    nav_msgs::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, "fused", 0.0, 100.0, occupancy_grid);
    global_occ_grid_pub_.publish(occupancy_grid);
  }

  if (grid_map_pub_.getNumSubscribers() >0 )
  {
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(global_map_, grid_map_msg);
    grid_map_pub_.publish(grid_map_msg);
  }
}
