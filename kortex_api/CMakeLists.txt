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
link_libraries(lib/release/libKortexApiCpp.a)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#find_library(${PROJECT_NAME} NAMES KortexApiCpp PATHS "lib/release")
#catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME}
#)
#
#add_library( ${PROJECT_NAME} EXCLUDE_FROM_ALL ${PVAPI} )
#add_custom_command( TARGET PvAPI POST_BUILD COMMAND cp ${PVAPI} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ )
#set_target_properties(PvAPI PROPERTIES LINKER_LANGUAGE CXX )
#
#install( DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#
#install( FILES ${PVAPI}
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})