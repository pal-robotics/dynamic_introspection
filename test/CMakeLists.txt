add_executable(singletonTest singletonTest.cpp)
target_link_libraries(singletonTest ${catkin_LIBRARIES})

add_library(testClassA testClassA.cpp)
target_link_libraries(testClassA ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(testClassB testClassB.cpp)
target_link_libraries(testClassB ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(testClassPlugins testClassPlugins.cpp)
target_link_libraries(testClassPlugins testClassA testClassB ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(testClassPluginMain testClassPluginMain.cpp)
target_link_libraries(testClassPluginMain ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(testClassMain testClassMain.cpp)
target_link_libraries(testClassMain testClassA testClassB ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(thread_safe_test thread_safe/thread_safe_test.cpp)
target_link_libraries(thread_safe_test ${PROJECT_NAME} ${catkin_LIBRARIES})
