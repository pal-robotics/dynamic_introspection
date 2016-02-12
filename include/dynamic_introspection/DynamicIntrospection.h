///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _DYNAMIC_INTROSPECTION_
#define _DYNAMIC_INTROSPECTION_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_introspection/IntrospectionMsg.h>

/**
 * @brief The DynamicIntrospection class allows to do dynamic instrospection of different
 * c++ types through ros topics and rosbag
 */
class DynamicIntrospection{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static DynamicIntrospection* Instance();

  virtual ~DynamicIntrospection();

  void registerVariable(int *variable, std::string id);
  void registerVariable(double *variable, std::string id);
  void registerVariable(bool *variable, std::string id);
  //void registerVariable(Eigen::Map<const Eigen::Vector3d> *variable, std::string id);
  void registerVariable(Eigen::Vector3d *variable, std::string id);
  void registerVariable(Eigen::VectorXd *variable, std::string id);
  void registerVariable(Eigen::MatrixXd *variable, std::string id);

  void unRegisterVariable(int *variable, std::string id);
  void unRegisterVariable(double *variable, std::string id);
  void unRegisterVariable(bool *variable, std::string id);
  //void unRegisterVariable(Eigen::Map<const Eigen::Vector3d> *variable, std::string id);
  void unRegisterVariable(Eigen::Vector3d *variable, std::string id);
  void unRegisterVariable(Eigen::VectorXd *variable, std::string id);
  void unRegisterVariable(Eigen::MatrixXd *variable, std::string id);

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
  ros::Publisher introspectionPub_;

  rosbag::Bag bag_;

  //Registered variables
  std::vector< std::pair<std::string, int*> > registeredInt_;
  std::vector< std::pair<std::string, double*> > registeredDouble_;
  std::vector<std::pair<std::string, bool*> > registeredBool_;
  std::vector<std::pair<std::string, Eigen::VectorXd*> > registeredVector_;
  std::vector<std::pair<std::string, Eigen::Vector3d*> > registered3dVector_;
  //std::vector<std::pair<std::string, Eigen::Map<const Eigen::Vector3d>*> > registered3dMap_;
  std::vector<std::pair<std::string, Eigen::MatrixXd*> > registeredMatrix_;

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
