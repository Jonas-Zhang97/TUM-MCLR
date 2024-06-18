cmake_minimum_required(VERSION 3.0.2)
project(tutorial_4)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorial_4
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/tutorial_4/config.py
  src/tutorial_4/tsid_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
