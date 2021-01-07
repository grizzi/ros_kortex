cmake_minimum_required(VERSION 3.10)
project(kortex_api)

find_package(catkin REQUIRED COMPONENTS)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-reorder")

add_definitions(-D_OS_UNIX)
catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(lib/${CMAKE_BUILD_TYPE}/libKortexApiCpp.a)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


