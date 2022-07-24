cmake_minimum_required(VERSION 3.0.2)
project(robot_state_publisher)

include(CheckCXXCompilerFlag)
unset(COMPILER_SUPPORTS_CXX11 CACHE)
if(MSVC)
  # https://docs.microsoft.com/en-us/cpp/build/reference/std-specify-language-standard-version
  # MSVC will fail the following check since it does not have the c++11 switch
  # however, c++11 is always enabled (the newer /std:c++14 is enabled by default)
  check_cxx_compiler_flag(/std:c++11 COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    add_compile_options(/std:c++11)
  endif()

  # MSVC does not support the Wextra flag
  add_compile_options(/Wall)
else()
  check_cxx_compiler_flag(-std=c++11 COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    add_compile_options(-std=c++11)
  endif()
  add_compile_options(-Wall -Wextra)
endif()

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS kdl_parser roscpp rosconsole rostime sensor_msgs tf2_ros tf2_kdl
)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver joint_state_listener
  INCLUDE_DIRS include
  DEPENDS kdl_parser orocos_kdl roscpp rosconsole rostime sensor_msgs tf2_ros tf2_kdl urdfdom_headers
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(joint_state_listener src/joint_state_listener.cpp)
target_link_libraries(joint_state_listener ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/robot_state_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver joint_state_listener ${orocos_kdl_LIBRARIES})

# Tests

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(test_default_constructor test/test_default_constructor.launch test/test_default_constructor.cpp)
  target_link_libraries(test_default_constructor joint_state_listener ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_one_link ${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch test/test_one_link.cpp)
  target_link_libraries(test_one_link ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_two_links_fixed_joint ${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch test/test_two_links_fixed_joint.cpp)
  target_link_libraries(test_two_links_fixed_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_two_links_moving_joint ${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch test/test_two_links_moving_joint.cpp)
  target_link_libraries(test_two_links_moving_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_frames_and_slashes ${CMAKE_CURRENT_SOURCE_DIR}/test/test_frames_and_slashes.launch test/test_frames_and_slashes.cpp)
  target_link_libraries(test_frames_and_slashes ${catkin_LIBRARIES})

  add_rostest_gtest(test_joint_states_bag ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_states_bag.launch test/test_joint_states_bag.cpp)
  target_link_libraries(test_joint_states_bag ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_subclass ${CMAKE_CURRENT_SOURCE_DIR}/test/test_subclass.launch test/test_subclass.cpp)
  target_link_libraries(test_subclass ${catkin_LIBRARIES} ${PROJECT_NAME}_solver joint_state_listener)

  install(FILES test/one_link.urdf test/pr2.urdf test/two_links_fixed_joint.urdf test/two_links_moving_joint.urdf test/frames_and_slashes.urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

endif()

# Install library
install(TARGETS ${PROJECT_NAME}_solver joint_state_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Install executable
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})