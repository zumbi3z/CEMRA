cmake_minimum_required(VERSION 2.8.3)
project(magabot)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  rosserial_arduino
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
  add_message_files(
    FILES
    Encoders.msg
    Velocity.msg
    Color.msg
    Status.msg
    Control.msg
    Topics.msg
  )

## Generate Services in the 'srv' folder
  add_service_files(
    FILES
    StartRosbag.srv
    StopRosbag.srv
    CodeExec.srv
    )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

###################################
## catkin specific configuration ##
###################################

catkin_package(
	LIBRARIES magabot
	CATKIN_DEPENDS tf roscpp rospy rosserial_arduino joy std_msgs message_runtime #geometry_msgs message_generation
)

#catkin_package()

###########
## Build ##
###########

#include_directories( ${catkin_INCLUDE_DIRS} )

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(Odometry src/Odometry.cpp)
#add_dependencies(Odometry magabot_generate_messages_cpp)
#target_link_libraries(Odometry ${catkin_LIBRARIES})

#add_executable(Controller src/Controller.cpp)
#add_dependencies(Controller magabot_generate_messages_cpp)
#target_link_libraries(Controller ${catkin_LIBRARIES})

#add_executable(Sensors src/Sensors.cpp)
#add_dependencies(Sensors magabot_generate_messages_cpp)
#target_link_libraries(Sensors ${catkin_LIBRARIES})

add_executable(magabot_node src/magabot_node.cpp)
target_link_libraries(magabot_node ${catkin_LIBRARIES} )

