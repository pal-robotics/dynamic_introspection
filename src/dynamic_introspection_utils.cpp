#include <dynamic_introspection/dynamic_introspection_utils.h>
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
  DoesNotExistingVariableExceptionUtils(const unsigned index, const std::string &name,
                                        IntrospectionBagReader *br)
    : index_(index), std::runtime_error(""), br_(br), variable_name_(name)
  {
  }

  const char *what() const throw()
  {
    std::stringstream ss;
    ss << "Variable does not exist: " << variable_name_ << std::endl;
    ss << "Existing variables: " << std::endl;
    for (auto it = br_->doubleNameMap_[index_].begin();
         it != br_->doubleNameMap_[index_].end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    for (auto it = br_->intNameMap_[index_].begin(); it != br_->intNameMap_[index_].end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    for (auto it = br_->boolNameMap_[index_].begin(); it != br_->boolNameMap_[index_].end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    ROS_ERROR_STREAM(ss.str());
    return ss.str().c_str();
  }

  unsigned int index_;
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

  nMessages_ = view.size();

  ROS_INFO_STREAM("Number of messages: " << nMessages_);

  int counter = 0;

  boolNameMap_.resize(nMessages_);
  intNameMap_.resize(nMessages_);
  doubleNameMap_.resize(nMessages_);

  foreach (rosbag::MessageInstance const m, view)
  {
    dynamic_introspection::IntrospectionMsg::ConstPtr s =
        m.instantiate<dynamic_introspection::IntrospectionMsg>();

    boolValues_.resize(s->bools.size());
    for (size_t i = 0; i < s->bools.size(); ++i)
    {
      boolValues_[i].reserve(nMessages_);
      boolNameMap_[counter][s->bools[i].name] = i;
    }
    intValues_.resize(s->ints.size());
    for (size_t i = 0; i < s->ints.size(); ++i)
    {
      intValues_[i].reserve(nMessages_);
      intNameMap_[counter][s->ints[i].name] = i;
    }
    doubleValues_.resize(s->doubles.size());
    for (size_t i = 0; i < s->doubles.size(); ++i)
    {
      doubleValues_[i].reserve(nMessages_);
      doubleNameMap_[counter][s->doubles[i].name] = i;
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
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "Reading percentage of data: "
                                 << ((double)counter / (double)nMessages_) * 100. << " %");
  }

  ROS_INFO_STREAM("Finished reading bag");
}

unsigned int IntrospectionBagReader::getNumberMessages()
{
  return nMessages_;
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<bool> &value, const bool throw_not_existing)
{
  value.reserve(nMessages_);

  for (size_t i = 0; i < nMessages_; ++i)
  {
    int index = -1;
    if (!getMapValue(boolNameMap_[i], variableId, index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId, this);
    }
    if (index >= 0)
    {
      value.push_back(boolValues_[index][i]);
    }
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<double> &value, const bool throw_not_existing)
{
  value.reserve(nMessages_);

  for (size_t i = 0; i < nMessages_; ++i)
  {
    int index = -1;
    if (!getMapValue(doubleNameMap_[i], variableId, index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId, this);
    }
    if (index >= 0)
    {
      value.push_back(doubleValues_[index][i]);
    }
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<Eigen::Vector3d> &value,
                                         const bool throw_not_existing)
{
  std::vector<std::string> ids;
  ids.push_back(variableId + "_X");
  ids.push_back(variableId + "_Y");
  ids.push_back(variableId + "_Z");
  getVariable(ids, value, throw_not_existing);
}

void IntrospectionBagReader::getVariable(const std::vector<std::string> &variableId,
                                         std::vector<Eigen::Vector3d> &value,
                                         const bool throw_not_existing)
{
  value.reserve(nMessages_);

  assert(variableId.size() == 3);

  for (size_t i = 0; i < nMessages_; ++i)
  {
    int index1 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[0], index1) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[0], this);
    }
    int index2 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[1], index2) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[1], this);
    }
    int index3 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[2], index3) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[2], this);
    }

    if ((index1 >= 0) && (index2 >= 0) && (index3 >= 0))
    {
      Eigen::Vector3d v(doubleValues_[index1][i], doubleValues_[index2][i],
                        doubleValues_[index3][i]);
      //      nanDetectedEigen(v);
      for (size_t i = 0; i < 3; ++i)
      {
        if (std::isnan(v[i]) || std::isinf(v[i]))
        {
          ROS_WARN_STREAM("found nan while parsing variable: " << variableId[i]);
          v[i] = 0.;
        }
      }
      value.push_back(v);
    }
  }
}


void IntrospectionBagReader::getVariable(const std::string &variable_id,
                                         std::vector<Eigen::Quaterniond> &value,
                                         const bool throw_not_existing)
{
  value.reserve(nMessages_);

  for (size_t i = 0; i < nMessages_; ++i)
  {
    int index1 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QX", index1) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QX", this);
    }
    int index2 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QY", index2) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QY", this);
    }
    int index3 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QZ", index3) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QZ", this);
    }

    int index4 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QW", index4) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QW", this);
    }

    if ((index1 >= 0) && (index2 >= 0) && (index3 >= 0) && (index4 >= 0))
    {
      value.push_back(Eigen::Quaterniond(doubleValues_[index4][i], doubleValues_[index1][i],
                                         doubleValues_[index2][i], doubleValues_[index3][i]));
    }
  }
}

/*
void IntrospectionBagReader::getVariable(const std::vector<std::string> &names,
                                         std::vector<Eigen::VectorXd> &value,
                                         const bool throw_not_existing)
{
  Eigen::VectorXd temp(names.size());
  value.reserve(nMessages_);
  for (size_t i = 0; i < names.size(); ++i)
  {
    assert(nMessages_ == value[i].size());

    int index;
    if (!getMapValue(doubleNameMap_, names[i], index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(names[i], this);
    }

    for (size_t j = 0; j < nMessages_; ++j)
    {
      temp(j) = doubleValues_[i][j];
    }

    value.push_back(temp);
  }
}
*/
}
