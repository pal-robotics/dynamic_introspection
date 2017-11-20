#ifndef _DYNAMIC_INTROSPECTION_UTILS_
#define _DYNAMIC_INTROSPECTION_UTILS_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <Eigen/Dense>

namespace dynamic_introspection
{
class IntrospectionBagReader
{
public:
  IntrospectionBagReader(const std::string &packageName, const std::string &bagFileName,
                         const std::string introspection_topic_name);

  IntrospectionBagReader(const std::string &bagFileName,
                         const std::string introspection_topic_name);

  void readBag(rosbag::Bag &bag);

  unsigned int getNumberMessages();

  void getVariable(const std::string &variableId, std::vector<bool> &value);

  void getVariable(const std::string &variableId, std::vector<double> &value);

  void getVariable(const std::string &variableId1, const std::string &variableId2,
                   const std::string &variableId3, std::vector<Eigen::Vector3d> &value);

  void getVariable(const std::string &variableId1, const std::string &variable2,
                   const std::string &variable3, const std::string &variableId4,
                   std::vector<Eigen::Quaterniond> &value);

  void getVariable(const std::vector<std::string> &names, std::vector<Eigen::VectorXd> &value);

  std::map<std::string, int> intNameMap_;
  std::map<std::string, int> doubleNameMap_;
  std::map<std::string, int> boolNameMap_;

private:
  std::string introspection_topic_name_;

  unsigned int nMessages_;

  std::vector<std::vector<int> > intValues_;
  std::vector<std::vector<double> > doubleValues_;
  std::vector<std::vector<bool> > boolValues_;
};
}

#endif
