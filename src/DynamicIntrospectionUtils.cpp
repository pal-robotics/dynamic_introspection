#include <dynamic_introspection/DynamicIntrospectionUtils.h>
#include <rosbag/view.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace dynamic_introspection
{
// Returns if the element exists in the map and if it does it returns the element
template <class Key, class Value, class Comparator, class Alloc>
bool getMapValue(const std::map<Key, Value, Comparator, Alloc> &my_map, Key key, Value &out)
{
  typename std::map<Key, Value, Comparator, Alloc>::const_iterator it = my_map.find(key);
  if (it != my_map.end())
  {
    out = it->second;
    return true;
  }
  return false;
}

struct DoesNotExistingVariableExceptionUtils : public std::runtime_error
{
  DoesNotExistingVariableExceptionUtils(const std::string &name, IntrospectionBagReader *br)
    : std::runtime_error(""), br_(br), variable_name_(name)
  {
  }

  const char *what() const throw()
  {
    std::stringstream ss;
    ss << "Variable does not exist: " << variable_name_ << std::endl;
    ss << "Existing variables: " << std::endl;
    for (auto it = br_->doubleNameMap_.begin(); it != br_->doubleNameMap_.end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    for (auto it = br_->intNameMap_.begin(); it != br_->intNameMap_.end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    for (auto it = br_->boolNameMap_.begin(); it != br_->boolNameMap_.end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    ROS_ERROR_STREAM(ss.str());
    return ss.str().c_str();
  }

  std::string variable_name_;
  IntrospectionBagReader *br_;
};

IntrospectionBagReader::IntrospectionBagReader(const std::string &packageName,
                                               const std::string &bagFileName,
                                               const std::string introspection_topic_name)
  : introspection_topic_name_(introspection_topic_name)
{
  ROS_INFO_STREAM("Reading bag: " << bagFileName);

  rosbag::Bag bag;
  bag.open(bagFileName, rosbag::bagmode::Read);

  readBag(bag);
}

IntrospectionBagReader::IntrospectionBagReader(const std::string &bagFileName,
                                               const std::string introspection_topic_name)
  : introspection_topic_name_(introspection_topic_name)
{
  ROS_INFO_STREAM("Reading bag: " << bagFileName);

  rosbag::Bag bag;
  bag.open(bagFileName, rosbag::bagmode::Read);

  readBag(bag);
}

void IntrospectionBagReader::readBag(rosbag::Bag &bag)
{
  std::vector<std::string> topics;
  topics.push_back(std::string(introspection_topic_name_));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool firstMessage = true;

  nMessages_ = view.size();

  ROS_INFO_STREAM("Number of messages: " << nMessages_);

  int counter = 0;

  foreach (rosbag::MessageInstance const m, view)
  {
    dynamic_introspection::IntrospectionMsg::ConstPtr s =
        m.instantiate<dynamic_introspection::IntrospectionMsg>();

    if (firstMessage)
    {
      ROS_INFO_STREAM("Number of Bool parameters: " << s->bools.size());

      boolValues_.resize(s->bools.size());
      for (size_t i = 0; i < s->bools.size(); ++i)
      {
        boolValues_[i].resize(nMessages_);
        boolNameMap_[s->bools[i].name] = i;
      }

      ROS_INFO_STREAM("Number of Int parameters: " << s->ints.size());
      intValues_.resize(s->ints.size());
      for (size_t i = 0; i < s->ints.size(); ++i)
      {
        intValues_[i].resize(nMessages_);
        intNameMap_[s->ints[i].name] = i;
      }

      ROS_INFO_STREAM("Number of Double parameters: " << s->doubles.size());
      doubleValues_.resize(s->doubles.size());
      for (size_t i = 0; i < s->doubles.size(); ++i)
      {
        doubleValues_[i].resize(nMessages_);
        doubleNameMap_[s->doubles[i].name] = i;
      }

      firstMessage = false;
    }

    for (size_t i = 0; i < boolValues_.size(); ++i)
    {
      boolValues_[i][counter] = s->bools[i].value;
    }

    for (size_t i = 0; i < intValues_.size(); ++i)
    {
      intValues_[i][counter] = s->ints[i].value;
    }

    for (size_t i = 0; i < doubleValues_.size(); ++i)
    {
      doubleValues_[i][counter] = s->doubles[i].value;
    }

    ++counter;
    ROS_INFO_STREAM_THROTTLE(
        1.0, "Reading percentage of data: " << ((double)counter / (double)nMessages_) * 100. << " %");
  }

  ROS_INFO_STREAM("Finished reading bag");
}

unsigned int IntrospectionBagReader::getNumberMessages()
{
  return nMessages_;
}

void IntrospectionBagReader::getVariable(const std::string &variableId, std::vector<bool> &value)
{
  assert(nMessages_ == value.size());
  value.resize(nMessages_);
  int index;
  if (!getMapValue(boolNameMap_, variableId, index))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId, this);
  }

  for (size_t i = 0; i < nMessages_; ++i)
  {
    value[i] = boolValues_[index][i];
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId, std::vector<double> &value)
{
  assert(nMessages_ == value.size());
  value.resize(nMessages_);
  int index;
  if (!getMapValue(doubleNameMap_, variableId, index))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId, this);
  }

  for (size_t i = 0; i < nMessages_; ++i)
  {
    value[i] = doubleValues_[index][i];
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId1,
                                         const std::string &variableId2,
                                         const std::string &variableId3,
                                         std::vector<Eigen::Vector3d> &value)
{
  assert(nMessages_ == value.size());
  value.resize(nMessages_);
  int index1;
  if (!getMapValue(doubleNameMap_, variableId1, index1))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId1, this);
  }
  int index2;
  if (!getMapValue(doubleNameMap_, variableId2, index2))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId2, this);
  }
  int index3;
  if (!getMapValue(doubleNameMap_, variableId3, index3))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId3, this);
  }

  for (size_t i = 0; i < nMessages_; ++i)
  {
    value[i] = Eigen::Vector3d(doubleValues_[index1][i], doubleValues_[index2][i],
                               doubleValues_[index3][i]);
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId1,
                                         const std::string &variableId2,
                                         const std::string &variableId3,
                                         const std::string &variableId4,
                                         std::vector<Eigen::Quaterniond> &value)
{
  assert(nMessages_ == value.size());
  value.resize(nMessages_);
  int index1;
  if (!getMapValue(doubleNameMap_, variableId1, index1))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId1, this);
  }
  int index2;
  if (!getMapValue(doubleNameMap_, variableId2, index2))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId2, this);
  }
  int index3;
  if (!getMapValue(doubleNameMap_, variableId3, index3))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId3, this);
  }

  int index4;
  if (!getMapValue(doubleNameMap_, variableId4, index4))
  {
    throw DoesNotExistingVariableExceptionUtils(variableId3, this);
  }

  for (size_t i = 0; i < nMessages_; ++i)
  {
    value[i] = Eigen::Quaterniond(doubleValues_[index4][i], doubleValues_[index1][i],
                                  doubleValues_[index2][i], doubleValues_[index3][i]);
  }
}

void IntrospectionBagReader::getVariable(const std::vector<std::string> &names,
                                         std::vector<Eigen::VectorXd> &value)
{
  for (size_t i = 0; i < names.size(); ++i)
  {
    assert(nMessages_ == value[i].size());

    int index;
    if (!getMapValue(doubleNameMap_, names[i], index))
    {
      throw DoesNotExistingVariableExceptionUtils(names[i], this);
    }

    for (size_t j = 0; j < nMessages_; ++j)
    {
      value[i](j) = doubleValues_[i][j];
    }
  }
}
}
