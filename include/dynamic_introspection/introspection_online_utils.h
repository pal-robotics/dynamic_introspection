#ifndef _INTROSPECTION_ONLINE_UTILS_
#define _INTROSPECTION_ONLINE_UTILS_

#include <ros/ros.h>
#include <dynamic_introspection/dynamic_introspection_utils.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <boost/scoped_ptr.hpp>
#include <ros/callback_queue.h>

namespace dynamic_introspection
{
class IntrospectionOnlineReader : public IntrospectionBagReader
{
public:
  IntrospectionOnlineReader(const std::string &topic = "/introspection_data");

  virtual ~IntrospectionOnlineReader();

  void start();

  void stop();

private:
  ros::NodeHandle nh_;
  ros::CallbackQueue cb_queue_;
  boost::scoped_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber sub_;

  void introspectionCB(const IntrospectionMsgConstPtr &msg);
};
}

#endif
