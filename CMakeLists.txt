cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	tf2_ros
	cv_bridge
	roscpp
	std_msgs
	image_transport
	eigen_conversions
)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

find_package(OpenCV)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavros
  CATKIN_DEPENDS roscpp geometry_msgs eigen_conversions 
  DEPENDS Boost
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(calibration)

###########
##install##
###########

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
