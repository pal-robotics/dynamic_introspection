#ifndef _DYNAMIC_INTROSPECTION_UTILS_
#define _DYNAMIC_INTROSPECTION_UTILS_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <Eigen/Dense>

class IntrospectionBagReader{

public:

  IntrospectionBagReader(const std::string &packageName, const std::string &bagFileName);

  IntrospectionBagReader(const std::string &bagFileName);

  void readBag(rosbag::Bag &bag);

  int getNumberMessages();

  void getVariable(const std::string &variableId, std::vector<bool> &value);

  void getVariable(const std::string &variableId, std::vector<double> &value);

  void getVariable(const std::string &variableId1, const std::string &variableId2, const std::string &variableId3,
                   std::vector<Eigen::Vector3d> &value);

  void getVariable(const std::string &variableId1, const std::string &variable2,
                   const std::string &variable3, const std::string &variableId4,
                   std::vector<Eigen::Quaterniond> &value);

  void getVariable(const std::vector<std::string> &names, std::vector<Eigen::VectorXd> &value);

private:

  int nMessages_;

//  std::vector<std::pair<std::string, std::vector<int> > > intValues_;
//  std::vector<std::pair<std::string, std::vector<double> > > doubleValues_;
//  std::vector<std::pair<std::string, std::vector<bool> > > boolValues_;

  std::vector<std::vector<int>  > intValues_;
  std::vector<std::vector<double> > doubleValues_;
  std::vector<std::vector<bool> > boolValues_;

  std::map<std::string, int> intNameMap_;
  std::map<std::string, int> doubleNameMap_;
  std::map<std::string, int> boolNameMap_;

};


#endif
