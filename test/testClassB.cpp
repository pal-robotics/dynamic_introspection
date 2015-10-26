#include "testClassB.h"
#include <dynamic_introspection/DynamicIntrospection.h>

TestClassB::TestClassB(){
  cont_ = 0;
  REGISTER_VARIABLE(&cont_, "counterB");
}

void TestClassB::update(){
  --cont_;
}
