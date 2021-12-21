cmake_minimum_required(VERSION 3.5)
project(point_transformation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory (../lib lib)

include_directories(
  include
  ${ros_core_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PixelToPoint.srv"
  DEPENDENCIES sensor_msgs geometry_msgs
)

add_executable(point_transformation_node src/point_transformation_node.cpp)
target_include_directories(point_transformation_node 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PUBLIC ../lib/include
)

ament_target_dependencies(point_transformation_node
  "rclcpp"
  "ros_core"
  "cv_bridge"
)

rosidl_target_interfaces(point_transformation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
target_link_libraries(point_transformation_node
  point_transformation_lib
  ros_core::ros_core_lib
  ${OpenCV_LIBS}
)
  
install(TARGETS 
  point_transformation_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
