#ifndef _TEST_CLASS_A_
#define _TEST_CLASS_A_

#include "testClassAbstract.h"

class TestClassA: public TestClassBase{
public:

  TestClassA();
  void update();

private:
  int cont_;

};

#endif
