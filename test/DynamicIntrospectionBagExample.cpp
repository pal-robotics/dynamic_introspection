#include <ros/ros.h>
#include <dynamic_introspection/DynamicIntrospection.h>

int main(int argc, char **argv) {
  // Setup debugging rosconsole
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");



  bool bool_test = false;

 // DynamicIntrospection di(nh, "debug_test");

  REGISTER_VARIABLE(&bool_test, "bool_test")

  Eigen::MatrixXd matrix_test(5,5);
  matrix_test.setIdentity();
  REGISTER_VARIABLE(&matrix_test, "matrix_test")

  Eigen::VectorXd vector_test(5);
  vector_test.setRandom();
  REGISTER_VARIABLE(&vector_test, "vector_test")

  Eigen::VectorXd vector_test2(5);
  vector_test2.setRandom();
  REGISTER_VARIABLE(&vector_test2, "vector_test2")

  Eigen::VectorXd vector_test3(5);
  vector_test3.setRandom();
  REGISTER_VARIABLE(&vector_test3, "vector_test3")

  ROS_INFO("Spinning node");

  OPEN_BAG("test.bag")
  PUBLISH_DEBUG_DATA_BAG
  CLOSE_BAG


  return 0;
}
