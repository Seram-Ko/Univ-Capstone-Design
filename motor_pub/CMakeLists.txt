cmake_minimum_required(VERSION 3.0.2)
project(motor_pub)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor_pub
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(motor_pub src/motor_pub.cpp)
add_dependencies(motor_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


