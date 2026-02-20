cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_square_mission)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs std_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})

# ==================== C++ Node ====================
add_executable(square_mission_cpp src/square_mission.cpp)
target_link_libraries(square_mission_cpp ${catkin_LIBRARIES})

# ==================== Python Node ====================
catkin_install_python(PROGRAMS
  scripts/square_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ==================== Install ====================
install(TARGETS square_mission_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
