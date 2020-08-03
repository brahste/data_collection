/* Subscribes to an integer value at a specified frequency
 * Author: B. Stefanuk
 * Date: July 31, 2020
 */

#include "ros/ros.h"
#include "std_msgs/Int8.h"

// Callback function for reception of a message
void terrainClassCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("[Listener] Received terrain class %d\n", msg->data);
}

// Recall, main() is the primary access point of an executable into the programce
int main(int argc, char **argv)
{
  // Initiate new ROS node called class_recorder_node
  ros::init(argc, argv, "class_recorder_node");
  // Create a node handle
  ros::NodeHandle nodehandle;
  // Create a publisher with a topic "terrain_class" that will send an Int8 message
  ros::Subscriber terrain_class_subscriber = nodehandle.subscribe("terrain_class", 100, terrainClassCallback);
  // Enter a loop, running callbacks
  ros::spin();

  return 0;
}
    
