#include "vectornav/vectornav.hpp"

int main(int argc, char const *argv[])
{
  ros::NodeHandle pnh("~");
  vectornav::Config config;
  vectornav::VectorNav vn(config, pnh);
  vn.StartSensor();
  ros::spin();
  vn.StopSensor();
  return 0;
}
