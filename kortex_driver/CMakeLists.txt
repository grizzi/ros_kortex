cmake_minimum_required(VERSION 2.8.3)
project(kortex_driver)

add_compile_options(-std=c++17)
add_definitions(-D_OS_UNIX)


set(API_LIB /home/giuseppe/kinova_ws/src/ros_kortex/kortex_api/lib/release/libKortexApiCpp.a)

set(PACKAGE_DEPENDENCIES
  kortex_api
  roscpp
  rospy
  std_msgs
  std_srvs
  control_toolbox
  controller_manager_msgs
  message_generation
  actionlib
  control_msgs
  urdf
  moveit_ros_planning_interface
  controller_manager
  hardware_interface
  angles)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/generated/robot/*.cpp" "src/generated/simulation/*.cpp")
file(GLOB_RECURSE non_generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/driver/*.cpp")
file(GLOB_RECURSE test_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/tests/*.cc")

# Find all auto-generated subdirectories in msg/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/msg/generated ${PROJECT_SOURCE_DIR}/msg/generated/*)
set(msg_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/msg/generated/${child})
      list(APPEND msg_generated_dir_list ${child})
    endif()
endforeach()

# Find all auto-generated subdirectories in srv/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/srv/generated ${PROJECT_SOURCE_DIR}/srv/generated/*)
set(srv_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/srv/generated/${child})
      list(APPEND srv_generated_dir_list ${child})
    endif()
endforeach()

## declare ROS messages and services
add_message_files(DIRECTORY msg/non_generated)
add_message_files(DIRECTORY msg/generated)
foreach(sub_dir ${msg_generated_dir_list})
    add_message_files(DIRECTORY msg/generated/${sub_dir})
endforeach()

add_service_files(DIRECTORY srv/non_generated)
foreach(sub_dir ${srv_generated_dir_list})
    add_service_files(DIRECTORY srv/generated/${sub_dir})
endforeach()

## generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## declare a catkin package
##catkin_package()
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES kortex_arm_driver_implementation
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
        DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)

# Generated files library
add_library(kortex_driver_generated_files ${generated_files})
target_link_libraries(kortex_driver_generated_files ${API_LIB} ${catkin_LIBRARIES} gcov)
add_dependencies(kortex_driver_generated_files ${PROJECT_NAME}_gencpp)

# Remove deprecation warnings from Kortex API
target_compile_options(kortex_driver_generated_files PRIVATE "-Wno-deprecated-declarations")

# Non-generated files library (driver implementation)
add_library(kortex_arm_driver_implementation ${non_generated_files})
target_link_libraries(kortex_arm_driver_implementation ${catkin_LIBRARIES} gcov ${API_LIB} kortex_driver_generated_files yaml-cpp)
add_dependencies(kortex_arm_driver_implementation kortex_driver_generated_files)

# Entry point executable for the driver
add_executable(kortex_arm_driver src/non-generated/main.cpp)
target_link_libraries(kortex_arm_driver_implementation ${API_LIB} ${catkin_LIBRARIES} gcov kortex_driver_generated_files)
target_link_libraries(kortex_arm_driver ${catkin_LIBRARIES} gcov kortex_arm_driver_implementation)
add_dependencies(kortex_arm_driver kortex_arm_driver_implementation)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	catkin_add_gtest(kortex_arm_driver_func_tests ${test_files})
  target_link_libraries(kortex_arm_driver_func_tests ${catkin_LIBRARIES} kortex_arm_driver_implementation)
  add_dependencies(kortex_arm_driver_func_tests kortex_arm_driver_implementation)
endif()
