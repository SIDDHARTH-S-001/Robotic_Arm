cmake_minimum_required(VERSION 3.1.3)
project(robotic_arm_6dof)

find_package(catkin REQUIRED) # Should not add runtime dependencies

catkin_package(CATKIN_DEPENDS roscpp rospy) 

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(robot_model_and_states script/robot_model_and_states.cpp)
#target_link_libraries(robot_model_and_states ${catkin_LIBRARIES})
