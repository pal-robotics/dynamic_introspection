#include <dynamic_introspection/introspection_online_utils.h>

namespace dynamic_introspection
{
IntrospectionOnlineReader::IntrospectionOnlineReader(const std::string &topic)
{
  nh_.setCallbackQueue(&cb_queue_);
  spinner_.reset(new ros::AsyncSpinner(1, &cb_queue_));
  sub_ = nh_.subscribe(topic, 1000, &IntrospectionOnlineReader::introspectionCB, this);
}

IntrospectionOnlineReader::~IntrospectionOnlineReader()
{
}

void IntrospectionOnlineReader::start()
{
  spinner_->start();
}

void IntrospectionOnlineReader::stop()
{
  spinner_->stop();
}

void IntrospectionOnlineReader::introspectionCB(const IntrospectionMsgConstPtr &msg)
{
  addMsg(msg);
}
}
