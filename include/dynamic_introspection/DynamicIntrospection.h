///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _DYNAMIC_INTROSPECTION_
#define _DYNAMIC_INTROSPECTION_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <realtime_tools/realtime_publisher.h>

/**
 * @brief The DynamicIntrospection class allows to do dynamic instrospection of different
 * c++ types through ros topics and rosbag
 */
class DynamicIntrospection{

public:

  static DynamicIntrospection* Instance();

  virtual ~DynamicIntrospection();

  void registerVariable(int *variable, const std::string &id);
  void registerVariable(double *variable, const std::string &id);
  void registerVariable(bool *variable, const std::string &id);
  void registerVariable(visualization_msgs::MarkerArray *variable, const std::string &id);

  void unRegisterVariable(int *variable, const std::string &id);
  void unRegisterVariable(double *variable, const std::string &id);
  void unRegisterVariable(bool *variable, const std::string &id);
  void unRegisterVariable(visualization_msgs::MarkerArray *variable, const std::string &id);

  void setOutputTopic(const std::string &outputTopic);

  void generateMessage();

  void publishDataTopic();

  void publishDataBag();

  void closeBag();

  void openBag(std::string fileName);

private:
  static DynamicIntrospection* m_pInstance;

  DynamicIntrospection();
  //DynamicIntrospection(ros::NodeHandle &nh, const std::string &topic);

  bool openedBag_;
  bool configured_;

  ros::NodeHandle node_handle_;
  //ros::Publisher introspectionPub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<dynamic_introspection::IntrospectionMsg> > introspectionPub_;

  rosbag::Bag bag_;

  //Registered variables
  std::vector< std::pair<std::string, int*> > registeredInt_;
  std::vector< std::pair<std::string, double*> > registeredDouble_;
  std::vector<std::pair<std::string, bool*> > registeredBool_;
  std::vector<std::pair<std::string, visualization_msgs::MarkerArray*> > registeredMarkers_;

  dynamic_introspection::IntrospectionMsg introspectionMessage_;

};

typedef boost::shared_ptr<DynamicIntrospection> DynamicIntrospectionPtr;


#define REGISTER_VARIABLE(VARIABLE, ID)                               \
   DynamicIntrospection::Instance()->registerVariable(VARIABLE, ID);  \

#define UNREGISTER_VARIABLE(VARIABLE, ID)                               \
   DynamicIntrospection::Instance()->unRegisterVariable(VARIABLE, ID);  \

#define OPEN_BAG(BAG_NAME)                                            \
   DynamicIntrospection::Instance()->openBag(BAG_NAME);               \

#define PUBLISH_DEBUG_DATA_BAG                                        \
   DynamicIntrospection::Instance()->publishDataBag();                \

#define PUBLISH_DEBUG_DATA_TOPIC                                        \
   DynamicIntrospection::Instance()->publishDataTopic();                \

#define CLOSE_BAG                                                     \
   DynamicIntrospection::Instance()->closeBag();                      \

#define CONFIGURE_OUTPUT_TOPIC(TOPIC_NAME)                            \
   DynamicIntrospection::Instance()->setOutputTopic(TOPIC_NAME);      \


#endif
