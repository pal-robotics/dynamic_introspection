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

  DynamicIntrospection(ros::NodeHandle &nh, const std::string &topic);

  ~DynamicIntrospection();

  void registerVariable(int *variable, std::string id);

  void registerVariable(double *variable, std::string id);

  void registerVariable(bool *variable, std::string id);

  void registerVariable(Eigen::VectorXd *variable, std::string id);

  void registerVariable(Eigen::MatrixXd *variable, std::string id);


  void generateMessage();

  void publishDataTopic();

  void publishDataBag();

  void closeBag();

  void openBag(std::string fileName);

private:
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
  std::vector<std::pair<std::string, Eigen::MatrixXd*> > registeredMatrix_;

  dynamic_introspection::IntrospectionMsg introspectionMessage_;

};

typedef boost::shared_ptr<DynamicIntrospection> DynamicIntrospectionPtr;

#endif
