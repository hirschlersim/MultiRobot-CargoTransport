cmake_minimum_required(VERSION 3.10)
project(leader_vehicle)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  tf
  message_generation
  mymsg
  apriltag_ros
)
find_package(Eigen3 REQUIRED )




catkin_package(

CATKIN_DEPENDS roscpp message_runtime
)

LINK_DIRECTORIES( ${ipopt_LIBRARY_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(source_list
	src/main.cpp
	src/batch_solver.cpp
	
)
#
ADD_EXECUTABLE(leader_vehicle ${source_list})
add_dependencies(leader_vehicle ${catkin_EXPORTED_TARGETS}  )
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(leader_vehicle
    ${cppad_lib}
    /usr/local/lib/libipopt.so
    ${colpack_libs}
    ${catkin_LIBRARIES}
)


