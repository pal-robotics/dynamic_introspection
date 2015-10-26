#include <ros/ros.h>
#include <dynamic_introspection/DynamicIntrospection.h>
#include "testClassAbstract.h"
#include <pluginlib/class_loader.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  // Setup debugging rosconsole
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  boost::shared_ptr<TestClassBase> tA;
  boost::shared_ptr<TestClassBase> tB;

  pluginlib::ClassLoader<TestClassBase> loader("dynamic_introspection", "TestClassBase");

  try{
    tA = loader.createInstance("TestClassA");
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    std::vector<std::string> classes = loader.getRegisteredLibraries();
    for(unsigned int i=0; i<classes.size(); ++i){
      std::cerr<<classes[i]<<std::endl;
    }
    return false;
  }

  try{
    tB = loader.createInstance("TestClassB");
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    std::vector<std::string> classes = loader.getRegisteredLibraries();
    for(unsigned int i=0; i<classes.size(); ++i){
      std::cerr<<classes[i]<<std::endl;
    }
    return false;
  }

  ROS_INFO("Spinning node");

  while(nh.ok()){
    tA->update();
    tB->update();

    std::cerr<<"*********"<<std::endl;
    PUBLISH_DEBUG_DATA_TOPIC
        ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
