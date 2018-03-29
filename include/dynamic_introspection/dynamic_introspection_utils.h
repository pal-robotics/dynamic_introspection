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

  void getVariable(const std::string &variableId, std::vector<bool> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::string &variableId, std::vector<double> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::string &variableId, std::vector<Eigen::Vector3d> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::string &variable_id, std::vector<Eigen::Quaterniond> &value,
                   const bool throw_not_existing = true);

  //  void getVariable(const std::vector<std::string> &names, std::vector<Eigen::VectorXd>
  //  &value,
  //                   const bool throw_not_existing = true);

  std::vector<std::map<std::string, int>> intNameMap_;
  std::vector<std::map<std::string, int>> doubleNameMap_;
  std::vector<std::map<std::string, int>> boolNameMap_;

private:
  std::string introspection_topic_name_;

  unsigned int nMessages_;

  std::vector<std::vector<int>> intValues_;
  std::vector<std::vector<double>> doubleValues_;
  std::vector<std::vector<bool>> boolValues_;
};
}

#endif
