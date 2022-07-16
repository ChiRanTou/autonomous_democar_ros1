cmake_minimum_required(VERSION 3.0.2)
project(democar_telop)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 roscpp
 std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(democar_telop_node 
  src/democar_telop_keyboard.cpp 
  src/keyboard_control.cpp
)

target_link_libraries(democar_telop_node ${catkin_LIBRARIES})

