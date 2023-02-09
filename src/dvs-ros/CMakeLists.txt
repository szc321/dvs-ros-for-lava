cmake_minimum_required(VERSION 3.5)
project(dvs-ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(OpenCV COMPONENTS highgui REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ddsmetadata REQUIRED)

include_directories(include)

set(SOURCES
    src/prophesee_ros_publisher.cpp)
set(INCLUDES
  include/prophesee_ros_publisher.h)

set(dependencies
  cv_bridge
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  ddsmetadata
  image_transport
)

add_executable(prophesee_ros_publisher
src/prophesee_ros_publisher.cpp
)
ament_target_dependencies(prophesee_ros_publisher
                          ${dependencies})
target_link_libraries(prophesee_ros_publisher ${OPENCV_INCLUDE_DIRS} MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui opencv_highgui)

install(TARGETS
       prophesee_ros_publisher
       DESTINATION lib/${PROJECT_NAME})
ament_package()
