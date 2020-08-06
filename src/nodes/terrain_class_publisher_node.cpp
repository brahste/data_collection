#include <data_collection/terrain_class_publisher.h>

// Main access point of executable into terrain class node
int main(int argc, char **argv)
{
	// Set up ROS and node handle
	ros::init(argc, argv, "terrain_class_publisher");
	ros::NodeHandle nh;

	// Create a new data_collection::TerrainClassPublisher object
	data_collection::TerrainClassPublisher node(nh);

	// Manage the callback queue
	ros::spin();

	return 0;
}