#include <ros/ros.h>
#include <dynamic_introspection/DynamicIntrospection.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  bool bool_test = false;

  DynamicIntrospection di(nh, "debug_test");

  di.registerVariable(&bool_test, "bool_test");

  Eigen::MatrixXd matrix_test(5,5);
  matrix_test.setIdentity();
  di.registerVariable(&matrix_test, "matrix_test");

  Eigen::VectorXd vector_test(5);
  vector_test.setRandom();
  di.registerVariable(&vector_test, "vector_test");

  ROS_INFO("Spinning node");

  while(nh.ok()){
    std::cerr<<"*********"<<std::endl;
    di.publishData();
    ros::spinOnce();
    ros::Duration(0.1).sleep();

  }
  return 0;
}
