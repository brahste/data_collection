// Publishes an integer value at a specified frequency
#include <iostream>
#include <data_collection/terrain_class_publisher.h>

namespace data_collection
{
// Define the constructor of the class
TerrainClassPublisher::TerrainClassPublisher(ros::NodeHandle nh) : nh_(nh), terrain_int_(8), enable_(true)
{
    // First, initialize the dynmaic reconfigure server and set the callback
    dynamic_reconfigure::Server<data_collection::TerrainClassIntegerConfig>::CallbackType cb;
    cb = boost::bind(&TerrainClassPublisher::reconfigureCallback, this, _1, _2);
    dynamic_server_.setCallback(cb);

    // Declare variable that can be modified by launch file or command line
    double rate = 200.0;

    // Initialize node parameters from launch file or command line using a private node handle
    // This allows multiple instances to be run simultaneously with different parameters
    ros::NodeHandle pnh("~");
    pnh.param("terrain_int", terrain_int_, terrain_int_);
    pnh.param("rate", rate, rate);
    pnh.param("enable", enable_, enable_);

    // Create the publisher and topic to publish on
    if (enable_)
    {
      startPublisher();
      //startPublisher(pnh.getParam("rate", rate));
    }
    // Timer
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &TerrainClassPublisher::timerCallback, this);
}

void TerrainClassPublisher::startPublisher()
{
  double rate = rate;
  std::cout << rate << std::endl;
  pub_ = nh_.advertise<std_msgs::Int8>("terrain_class_integer", rate);
}

void TerrainClassPublisher::stopPublisher()
{
   pub_.shutdown();
}

void TerrainClassPublisher::reconfigureCallback(data_collection::TerrainClassIntegerConfig &config, uint32_t level __attribute__((unused)))
{
    // Assign the member variables new values according to which is input in the GUI
    terrain_int_ = config.terrain_int;

    // Ensure that node is still enabled
    if (enable_ != config.enable)
    {
        if (config.enable)
        {
            startPublisher();
        }
        else
        {
            stopPublisher();
        }
    }
    enable_ = config.enable;
}

void TerrainClassPublisher::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
    if (!enable_)
    {
        return;
    }

    std_msgs::Int8 msg;
    msg.data = terrain_int_;

    pub_.publish(msg);
}
}
