cmake_minimum_required(VERSION 3.0.2)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs

)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
 FILES
 obj.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(track
    src/track.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(track
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(camera
    src/camera.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(camera
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(gps
    src/gps.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(gps
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(record
    src/record.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(record
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})

add_executable(traj
    src/traj.cpp
    src/include/movement.cpp
    src/include/run_yolo.cpp)

target_link_libraries(traj
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})



