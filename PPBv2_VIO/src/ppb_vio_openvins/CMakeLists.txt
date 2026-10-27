cmake_minimum_required(VERSION 3.8)
project(ppb_vio_openvins)

# Built only with the OpenVINS workspace sourced (scripts/build_ppb_vio.sh).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ov_core REQUIRED)
find_package(ov_init REQUIRED)
find_package(ov_msckf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# ov_msckf headers are compiled for ROS 2.
add_definitions(-DROS_AVAILABLE=2)

add_executable(openvins_serial src/openvins_serial.cpp)
target_include_directories(openvins_serial PRIVATE ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS}
                           ${CERES_INCLUDE_DIRS})
ament_target_dependencies(openvins_serial rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs
                          sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs cv_bridge image_transport
                          ov_core ov_init ov_msckf)
target_link_libraries(openvins_serial ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
install(TARGETS openvins_serial DESTINATION lib/${PROJECT_NAME})

ament_package()
