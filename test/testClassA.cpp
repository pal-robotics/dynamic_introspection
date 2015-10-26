#include "testClassA.h"
#include <dynamic_introspection/DynamicIntrospection.h>

TestClassA::TestClassA(){
  cont_ = 0;
  REGISTER_VARIABLE(&cont_, "counterA");
}

void TestClassA::update(){
  ++cont_;
}
