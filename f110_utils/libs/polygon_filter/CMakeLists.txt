cmake_minimum_required(VERSION 3.10)
project(polygon_filter)

# 필요한 ROS 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


# catkin_package()로 export할 내용 지정
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polygon_filter_lib
  CATKIN_DEPENDS roscpp std_msgs
)

# include 디렉토리와 catkin include 경로 추가
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# 라이브러리 타겟 생성 (소스 파일 경로는 실제 파일 위치에 맞춰 조정)
add_library(polygon_filter_lib src/polygon_filter.cpp)
target_link_libraries(polygon_filter_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

# 타겟 설치 (export 시킬 때 EXPORT 이름 지정)
install(TARGETS polygon_filter_lib
  EXPORT polygon_filter_export
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# include 디렉토리 설치
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# export 파일 생성: 다른 패키지에서 find_package() 시 사용
install(EXPORT polygon_filter_export
  FILE polygon_filterTargets.cmake
  NAMESPACE polygon_filter::
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)