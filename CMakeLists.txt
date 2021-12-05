cmake_minimum_required(VERSION 3.0.2)
project(single_motor_gripper)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  dynamixel_workbench_controllers
  dynamixel_workbench_toolbox
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   close.srv
   open.srv
   toggle.srv
   partial.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gripper_control_node src/gripper_control.cpp)
target_link_libraries(gripper_control_node ${catkin_LIBRARIES})