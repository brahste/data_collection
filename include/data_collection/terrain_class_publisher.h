//#ifndef TERRAIN_CLASS_PUBLISHER_H
//#define TERRAIN_CLASS_PUBLISHER_H

// ROS specific headers
#include <ros/ros.h>
#include <ros/time.h>

// Message type headers
#include <std_msgs/Int8.h>

// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <data_collection/TerrainClassIntegerConfig.h>

namespace data_collection
{
class TerrainClassPublisher
{
	public:
		// Constructor
		explicit TerrainClassPublisher(ros::NodeHandle nh);

	private:
		// Callback function for dynamic reconfigure server
		// Set as reference the config parameter, undetstand better what the level is
		void reconfigureCallback(data_collection::TerrainClassIntegerConfig &config, uint32_t level);

		// Callback function for timing the publications
		void timerCallback(const ros::TimerEvent &event);

		// Start the publisher
		void startPublisher();

		// Stop the publisher
		void stopPublisher();

		// ROS node handle
		ros::NodeHandle nh_;

		// Variable used to access callback at selected frequency
		ros::Timer timer_;

		// Message publisher
		ros::Publisher pub_;

		// Dynamic reconfigure server
		dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig> dynamic_server_;

		// Actual terrain class integer 
		int terrain_int_;

		// Flag to enable/disable node
		bool enable_;
};
}