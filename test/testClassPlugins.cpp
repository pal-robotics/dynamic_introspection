#include "testClassA.h"
#include "testClassB.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(TestClassA, TestClassBase);
PLUGINLIB_EXPORT_CLASS(TestClassB, TestClassBase);
