cmake_minimum_required(VERSION 2.8.3)
project(dynamic_introspection)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation rosbag)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_message_files(DIRECTORY msg FILES BoolParameter.msg
                                      IntrospectionMsg.msg
                                      VectorParameter.msg
                                      DoubleParameter.msg
                                      MatrixParameter.msg
                                      IntParameter.msg
                                      StringParameter.msg)


generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rosbag
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/DynamicIntrospection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(DynamicIntrospectionExample test/DynamicIntrospectionExample.cpp)
target_link_libraries(DynamicIntrospectionExample ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(DynamicIntrospectionBagExample test/DynamicIntrospectionBagExample.cpp)
target_link_libraries(DynamicIntrospectionBagExample ${PROJECT_NAME} ${catkin_LIBRARIES})

add_subdirectory(singleton_tests)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
