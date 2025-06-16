cmake_minimum_required(VERSION 3.10)
project(grid_filter)

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_python_setup()

# Specify what to export via catkin_package()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_filter_lib
  CATKIN_DEPENDS roscpp std_msgs
)

# Add include directories and catkin include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Create library target (adjust source file path as needed)
add_library(grid_filter_lib src/grid_filter.cpp)
target_link_libraries(grid_filter_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

# Install target (specify EXPORT name for export)
install(TARGETS grid_filter_lib
  EXPORT grid_filter_export
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Generate export file: used by other packages with find_package()
install(EXPORT grid_filter_export
  FILE grid_filterTargets.cmake
  NAMESPACE grid_filter::
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/grid_filter/grid_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
