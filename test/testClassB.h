#ifndef _TEST_CLASS_B_
#define _TEST_CLASS_B_

#include "testClassAbstract.h"

class TestClassB: public TestClassBase{
public:

  TestClassB();
  void update();
private:
  int cont_;
};

#endif
