#include <ethz_grid_map_proc/grid_map_to_occupancy_grid.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grid_map_to_occupancy_grid_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  GridMapToOccupancyGrid grid_map_to_occ_grid(nh, pnh);

  ros::spin();

  return 0;
}






