#include "sonar_clearing_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"


PLUGINLIB_REGISTER_CLASS(LaserScanSonarFilter, laser_filters::LaserScanSonarFilter, filters::FilterBase<sensor_msgs::LaserScan>)
