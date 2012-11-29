#include "cwru_battery_analyzer/battery_analyzer.h"

using namespace diagnostic_aggregator;
using namespace std;

PLUGINLIB_REGISTER_CLASS(BatteryAnalyzer,  
                         diagnostic_aggregator::BatteryAnalyzer, 
                         diagnostic_aggregator::Analyzer)

BatteryAnalyzer::BatteryAnalyzer() :
  path_(""), nice_name_("Battery"), 
{ }

BatteryAnalyzer::~BatteryAnalyzer() { }

bool BatteryAnalyzer::init(const string base_name, const ros::NodeHandle &n)
{ 
  // path_ = BASE_NAME/Motors
  path_ = base_name + "/" + nice_name_;

  if (!n.getParam("power_board_name", power_board_name_))
  {
     ROS_ERROR("No power board name was specified in PR2MotorsAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
     return false;
  }

  // Make a "missing" item for the EtherCAT Master
  boost::shared_ptr<StatusItem> item(new StatusItem("EtherCAT Master"));
  eth_master_item_ = item;

  has_initialized_ = true;
  
  return true;
}

