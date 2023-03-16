cmake_minimum_required(VERSION 2.8.3)
project(e2box_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

set( sources
  src/main.cpp
  src/t_serial.cpp
  src/e2box_imu_9dofv4.cpp
)

set( headers
  include/t_serial.h
  include/e2box_imu_9dofv4.h
  include/main.h
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES e2box_imu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
  src/t_serial.cpp
  src/e2box_imu_9dofv4.cpp
  src/main.cpp
)

add_executable(${PROJECT_NAME}_node ${headers} ${sources})
add_executable(t_serial ${headers} ${sources})
add_executable(e2box_imu_9dofv4 ${headers} ${sources})
add_executable(main ${headers} ${sources})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
target_link_libraries(t_serial
   ${catkin_LIBRARIES}
)
target_link_libraries(e2box_imu_9dofv4
   ${catkin_LIBRARIES}
)
target_link_libraries(main
   ${catkin_LIBRARIES}
)