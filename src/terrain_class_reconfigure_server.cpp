#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "data_collection/TerrainClassIntegerConfig.h"

void callback(data_collection::TerrainClassIntegerConfig &config, int8_t level)
{
  ROS_INFO("Reconfigure request: %d", config.terrain_class_int);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "class_reconfigure_server");
  dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig> server;
  dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

  
