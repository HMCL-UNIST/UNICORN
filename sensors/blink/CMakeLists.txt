cmake_minimum_required(VERSION 2.8.3)
project(blink1)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation 
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")

add_message_files(
  FILES
  Blink1msg.msg
)

add_service_files(
  FILES
  Blink.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(blink1_node src/blink1_node.cpp src/hiddata.cpp)
target_link_libraries(blink1_node usb ${catkin_LIBRARIES})
add_dependencies(blink1_node blink1_gencpp)

add_executable(blink1_example examples/blink1_example.cpp)
target_link_libraries(blink1_example ${catkin_LIBRARIES})
add_dependencies(blink1_example blink1_gencpp) 

add_executable(blink1_example_topic examples/blink1_example_topic.cpp)
target_link_libraries(blink1_example_topic ${catkin_LIBRARIES})
add_dependencies(blink1_example_topic blink1_gencpp) 


