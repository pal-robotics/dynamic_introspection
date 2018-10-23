///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _DYNAMIC_INTROSPECTION_
#define _DYNAMIC_INTROSPECTION_
#include <pal_statistics/pal_statistics_macros.h>
#include <rosbag/bag.h>
#include <Eigen/Dense>

namespace pal
{
template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                             Eigen::Vector3d *variable, RegistrationsRAII *bookkeeping,
                             bool enabled)
{
  /// only one id is returned, unregistration should be done with RegistrationRAII
  /// or vairable name
  registry.registerVariable(name + "_X", &variable->x(), bookkeeping, enabled);
  registry.registerVariable(name + "_Y", &variable->y(), bookkeeping, enabled);
  return registry.registerVariable(name + "_Z", &variable->z(), bookkeeping, enabled);  
}


template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                             Eigen::Quaterniond *variable, RegistrationsRAII *bookkeeping,
                             bool enabled)
{
  /// only one id is returned, unregistration should be done with RegistrationRAII
  /// or vairable name
  registry.registerVariable(name + "_QX", &variable->coeffs().x(), bookkeeping, enabled);
  registry.registerVariable(name + "_QY", &variable->coeffs().y(), bookkeeping, enabled);
  registry.registerVariable(name + "_QZ", &variable->coeffs().z(), bookkeeping, enabled);
  return registry.registerVariable(name + "_QW", &variable->coeffs().w(), bookkeeping, enabled);
}

// All these are hacks to preserve an old API
namespace statistics_bag_hack
{
static boost::shared_ptr<rosbag::Bag> bag_;
inline void statisticsOpenBag(const std::string &filename)
{
  bag_.reset(new rosbag::Bag);
  bag_->open(filename, rosbag::bagmode::Write);
}

inline void statisticsCloseBag()
{
  bag_->close();
  bag_.reset();
}

inline void statististicsWriteDataToBag()
{
  bag_->write("/introspection_data", ros::Time::now(),
              getRegistry("/introspection_data")->createMsg());
}
}  // namespace statistics_bag_hack
} // namespace pal

#define OPEN_BAG(BAG_NAME)                                            \
   pal::statistics_bag_hack::statisticsOpenBag(BAG_NAME);               \

#define PUBLISH_DEBUG_DATA_BAG                                        \
   pal::statistics_bag_hack::statististicsWriteDataToBag();                \

#define CLOSE_BAG                                                     \
   pal::statistics_bag_hack::statisticsCloseBag();                      \

#endif
