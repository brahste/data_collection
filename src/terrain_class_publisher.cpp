// Publishes an integer value at a specified frequency

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <dynamic_reconfigure/server.h>
#include "data_collection/TerrainClassIntegerConfig.h"


void reconfigureCallback(data_collection::TerrainClassIntegerConfig &config, int8_t level)
{
  ROS_INFO("Reconfigure request: %d", config.terrain_class_int);
}

// Recall, main() is the primary access point of an executable into the programce
int main(int argc, char **argv)
{
  // Initiate new ROS node called class_tracker_node
  ros::init(argc, argv, "reconfigureable_class_tracker_node");
  // Create a node handle
  ros::NodeHandle nodehandle;
  // Create a publisher with a topic "terrain_class" that will send an Int8 message
  ros::Publisher terrain_class_publisher = nodehandle.advertise<std_msgs::Int8>("terrain_class", 100);
  // Define the publishing rate
  // In the future it would be good to sync this with the publishing rate (or up to a scale factor)
  // of the publishing rate of the IMU
  ros::Rate loop_rate(10); // Publishes at 10 Hz

  int count = 0;
  while (ros::ok()) // Keep spinning until user Ctrl+C's
  {
    std_msgs::Int8 msg;

    msg.data = 0;

    ROS_INFO("[Publisher, %d] We are currently on terrain %d\n", count, msg.data);

    terrain_class_publisher.publish(msg);

    ros::spinOnce();

    loop_rate.sleep(); // Sleep for the rest of the cycle
    ++count;
  }
  return 0;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "class_reconfigure_server");
  dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig> server;
  dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig>::CallbackType f;

  f = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

      

   
  
